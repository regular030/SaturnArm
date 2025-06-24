#include "camera.h"
#include <iostream>

Camera::Camera(int width, int height) 
    : width(width), height(height) {}

bool Camera::init() {
    cap.open(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "ERROR: Failed to open camera device" << std::endl;
        return false;
    }
    
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap.set(cv::CAP_PROP_FPS, 15);
    
    if (!cap.isOpened()) {
        std::cerr << "ERROR: Failed to set camera parameters" << std::endl;
        return false;
    }
    
    initialized = true;
    std::cout << "Camera initialized at " << width << "x" << height << std::endl;
    return true;
}

bool Camera::capture_frame(std::vector<uchar>& buffer) {
    if (!initialized) return false;
    
    cv::Mat frame;
    if (!cap.read(frame)) {
        std::cerr << "WARNING: Failed to capture frame" << std::endl;
        return false;
    }
    
    if (frame.empty()) return false;
    
    cv::imencode(".jpg", frame, buffer, {cv::IMWRITE_JPEG_QUALITY, 80});
    return true;
}
