#include "arm_controller.h"
#include "camera.h"
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <atomic>
#include <iomanip>
#include <sstream>
#include <fstream>

namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;

constexpr int CAM_WIDTH = 640;
constexpr int CAM_HEIGHT = 480;
constexpr int CAM_FPS = 10;

std::atomic<ArmController*> active_arm{nullptr};
std::atomic<bool> running{true};

void send_http_response(tcp::socket& socket, http::status status, 
                        const std::string& content_type, const std::string& body) {
    http::response<http::string_body> res{status, 11};
    res.set(http::field::content_type, content_type);
    res.body() = body;
    res.prepare_payload();
    http::write(socket, res);
}

void send_websocket_message(beast::websocket::stream<tcp::socket>& ws, 
                            const std::string& message) {
    ws.write(net::buffer(message));
}

void handle_websocket(tcp::socket socket, ArmController& arm, Camera& camera) {
    try {
        beast::websocket::stream<tcp::socket> ws{std::move(socket)};
        ws.accept();
        
        active_arm = &arm;
        std::cout << "WebSocket connection established" << std::endl;
        
        beast::flat_buffer buffer;
        
        while (running) {
            // Read message
            buffer.clear();
            ws.read(buffer);
            std::string cmd = beast::buffers_to_string(buffer.data());
            
            // Process command
            if (cmd.find("move:") == 0) {
                float x, y;
                int z;
                if (sscanf(cmd.c_str(), "move:%f,%f,%d", &x, &y, &z) == 3) {
                    arm.move_to(x, y, z);
                }
            }
            else if (cmd == "calibrate") {
                arm.calibrate();
            }
            else if (cmd == "stop") {
                arm.emergency_stop();
            }
            
            // Send encoder positions
            std::ostringstream oss;
            oss << "encoders:"
                << arm.get_stepper_position() << ","
                << arm.get_base_position() << ","
                << arm.get_joint2_position() << ","
                << arm.get_claw_position();
            send_websocket_message(ws, oss.str());
            
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    } 
    catch (...) {
        std::cerr << "WebSocket connection closed" << std::endl;
        active_arm = nullptr;
    }
}

void handle_camera_stream(tcp::socket socket, Camera& camera) {
    if (!camera.is_initialized() && !camera.init()) {
        send_http_response(socket, http::status::internal_server_error, 
                           "text/plain", "Camera initialization failed");
        return;
    }
    
    try {
        beast::flat_buffer buffer;
        http::request<http::string_body> req;
        http::read(socket, buffer, req);
        
        http::response<http::string_body> res{
            http::status::ok,
            req.version(),
            "Streaming started"
        };
        res.set(http::field::content_type, "multipart/x-mixed-replace; boundary=frame");
        http::write(socket, res);
        
        std::cout << "Starting camera stream" << std::endl;
        
        while (running) {
            std::vector<uchar> jpeg_buffer;
            if (!camera.capture_frame(jpeg_buffer)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            
            std::ostringstream header;
            header << "--frame\r\n"
                   << "Content-Type: image/jpeg\r\n"
                   << "Content-Length: " << jpeg_buffer.size() << "\r\n\r\n";
            
            net::write(socket, net::buffer(header.str()));
            net::write(socket, net::buffer(jpeg_buffer));
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1000/CAM_FPS));
        }
    }
    catch (...) {
        std::cerr << "Camera stream terminated" << std::endl;
    }
}

void handle_http_request(tcp::socket socket, ArmController& arm, Camera& camera) {
    try {
        beast::flat_buffer buffer;
        http::request<http::string_body> req;
        http::read(socket, buffer, req);
        
        std::string target = req.target().to_string();
        std::cout << "HTTP Request: " << target << std::endl;
        
        if (target == "/") {
            // Serve simple HTML control interface
            std::string html = R"(
            <html><body>
            <h1>SaturnArm Control</h1>
            <a href="/camera">Camera Stream</a><br>
            <a href="/calibrate">Calibrate Arm</a><br>
            <a href="/stop">Emergency Stop</a>
            </body></html>
            )";
            send_http_response(socket, http::status::ok, "text/html", html);
        }
        else if (target == "/camera") {
            handle_camera_stream(std::move(socket), camera);
        }
        else if (target == "/calibrate") {
            arm.calibrate();
            send_http_response(socket, http::status::ok, "text/plain", "Calibration complete");
        }
        else if (target == "/stop") {
            arm.emergency_stop();
            send_http_response(socket, http::status::ok, "text/plain", "Emergency stop activated");
        }
        else if (target.find("/move") == 0) {
            float x = 0, y = 0;
            int z = 0;
            size_t pos = target.find('?');
            if (pos != std::string::npos) {
                std::string params = target.substr(pos + 1);
                sscanf(params.c_str(), "x=%f&y=%f&z=%d", &x, &y, &z);
                arm.move_to(x, y, z);
            }
            send_http_response(socket, http::status::ok, "text/plain", "Move command received");
        }
        else {
            send_http_response(socket, http::status::not_found, "text/plain", "404 Not Found");
        }
    }
    catch (...) {
        // Ignore connection errors
    }
}

int main() {
    // Skip WiFi credential retrieval
    std::cout << "Skipping WiFi credential retrieval" << std::endl;
    
    // Initialize hardware
    ArmController arm;
    if (!arm.init()) {
        std::cerr << "FATAL: Arm initialization failed" << std::endl;
        return 1;
    }

    Camera camera(CAM_WIDTH, CAM_HEIGHT);
    if (!camera.init()) {
        std::cerr << "WARNING: Camera initialization failed - streaming disabled" << std::endl;
    }

    // Create server
    net::io_context ioc;
    tcp::acceptor acceptor(ioc);
    acceptor.bind(tcp::endpoint(net::ip::make_address("0.0.0.0"), 80));
    acceptor.listen();
    
    std::cout << "SaturnArm control server running on port 80" << std::endl;

    while (running) {
        tcp::socket socket(ioc);
        acceptor.accept(socket);
        
        std::thread([s = std::move(socket), &arm, &camera]() mutable {
            try {
                // Create a stream wrapper for peeking
                beast::tcp_stream stream(std::move(s));
                
                // Peek at the first part of the request
                http::request_parser<http::empty_body> parser;
                parser.eager(false);
                
                beast::flat_buffer buffer;
                http::read_header(stream, buffer, parser);
                auto req = parser.get();
                
                std::cout << "Connection received for: " << req.target() << std::endl;
                
                if (req.target() == "/camera") {
                    handle_camera_stream(stream.release_socket(), camera);
                }
                else if (beast::websocket::is_upgrade(req)) {
                    handle_websocket(stream.release_socket(), arm, camera);
                }
                else {
                    handle_http_request(stream.release_socket(), arm, camera);
                }
            }
            catch (const std::exception& e) {
                std::cerr << "Connection error: " << e.what() << std::endl;
            }
            catch (...) {
                std::cerr << "Unknown connection error" << std::endl;
            }
        }).detach();
    }
    
    return 0;
}
