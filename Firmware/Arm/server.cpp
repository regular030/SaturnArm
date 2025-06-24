#include "arm_controller.h"
#include "camera.h"
#include "wifi_credentials.h"
#include <boost/beast.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <atomic>
#include <iomanip>
#include <sstream>

namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;

constexpr int CAM_WIDTH = 640;
constexpr int CAM_HEIGHT = 480;
constexpr int CAM_FPS = 10;

std::atomic<ArmController*> active_arm{nullptr};
std::atomic<bool> running{true};

void send_http_response(tcp::socket& socket, const std::string& message) {
    http::response<http::string_body> res{
        http::status::ok, 
        std::stoi("11")
    };
    res.set(http::field::content_type, "text/html");
    res.body() = "<html><body>" + message + "</body></html>";
    http::write(socket, res);
}

void handle_websocket(tcp::socket& socket, ArmController& arm) {
    beast::flat_buffer buffer;
    websocket::stream<tcp::socket> ws{std::move(socket)};
    
    try {
        ws.accept();
        active_arm = &arm;
        
        while (running) {
            // Read message
            buffer.clear();
            ws.read(buffer);
            
            // Process command
            std::string cmd = beast::buffers_to_string(buffer.data());
            if (cmd.find("move") != std::string::npos) {
                // Parse and execute move command
                // Format: move:x,y,z
            }
            
            // Send encoder positions
            std::ostringstream oss;
            oss << "{\"stepper\":" << arm.get_stepper_position()
                << ",\"base\":" << arm.get_base_position()
                << ",\"joint2\":" << arm.get_joint2_position()
                << ",\"claw\":" << arm.get_claw_position() << "}";
            ws.write(net::buffer(oss.str()));
            
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    } 
    catch (...) {
        active_arm = nullptr;
    }
}

void handle_camera_stream(tcp::socket& socket, Camera& camera) {
    http::response<http::string_body> res{
        http::status::ok,
        std::stoi("11")
    };
    res.set(http::field::content_type, "multipart/x-mixed-replace; boundary=frame");
    http::write(socket, res);
    
    while (running) {
        std::vector<uchar> jpeg_buffer;
        if (!camera.capture_frame(jpeg_buffer)) continue;
        
        std::ostringstream header;
        header << "--frame\r\n"
               << "Content-Type: image/jpeg\r\n"
               << "Content-Length: " << jpeg_buffer.size() << "\r\n\r\n";
        
        net::write(socket, net::buffer(header.str()));
        net::write(socket, net::buffer(jpeg_buffer));
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000/CAM_FPS));
    }
}

void handle_request(tcp::socket socket, ArmController& arm, Camera& camera) {
    try {
        beast::flat_buffer buffer;
        http::request<http::string_body> req;
        http::read(socket, buffer, req);
        
        if (req.method() == http::verb::get) {
            std::string target = req.target().to_string();
            
            if (target == "/camera") {
                handle_camera_stream(socket, camera);
            }
            else if (target.find("/move?") == 0) {
                // Parse and execute move command
                send_http_response(socket, "Move command received");
            }
            else if (target == "/encoders") {
                std::ostringstream oss;
                oss << "{\"stepper\":" << arm.get_stepper_position()
                    << ",\"base\":" << arm.get_base_position()
                    << ",\"joint2\":" << arm.get_joint2_position()
                    << ",\"claw\":" << arm.get_claw_position() << "}";
                send_http_response(socket, oss.str());
            }
            else if (target == "/calibrate") {
                arm.calibrate();
                send_http_response(socket, "Calibration complete");
            }
            else if (target == "/stop") {
                arm.emergency_stop();
                send_http_response(socket, "Emergency stop activated");
            }
            else {
                send_http_response(socket, "VR Arm Control Server Ready");
            }
        }
    }
    catch (...) {
        // Ignore errors
    }
}

int main() {
    ArmController arm;
    if (!arm.init()) {
        std::cerr << "Failed to initialize arm controller\n";
        return 1;
    }

    Camera camera(CAM_WIDTH, CAM_HEIGHT);
    if (!camera.init()) {
        std::cerr << "Failed to initialize camera\n";
        return 1;
    }

    net::io_context ioc;
    tcp::acceptor acceptor(ioc, {tcp::v4(), 80});
    
    std::thread control_thread([&]() {
        while (running) {
            if (auto* active = active_arm.load()) {
                // Send regular updates to active WebSocket clients
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });
    
    while (running) {
        tcp::socket socket(ioc);
        acceptor.accept(socket);
        std::thread([s = std::move(socket), &arm, &camera]() mutable {
            handle_request(std::move(s), arm, camera);
        }).detach();
    }
    
    control_thread.join();
    return 0;
}
