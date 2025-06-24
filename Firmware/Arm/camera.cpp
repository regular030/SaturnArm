#include "camera.h"
#include <iostream>

Camera::Camera(int width, int height) 
    : width(width), height(height) {}

bool Camera::init() {
    cap.open(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera\n";
        return false;
    }
    
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap.set(cv::CAP_PROP_FPS, 10);
    
    if (!cap.isOpened()) {
        std::cerr << "Failed to set camera parameters\n";
        return false;
    }
    
    return true;
}

bool Camera::capture_frame(std::vector<uchar>& buffer) {
    cv::Mat frame;
    if (!cap.read(frame) {
        std::cerr << "Failed to capture frame\n";
        return false;
    }
    
    if (frame.empty()) return false;
    
    cv::imencode(".jpg", frame, buffer);
    return true;
}
