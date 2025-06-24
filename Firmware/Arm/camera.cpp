#include "camera.h"
#include <iostream>

Camera::Camera(int width, int height) 
    : width(width), height(height) {}

bool Camera::init() {
    // Try multiple approaches to open camera
    for (int i = 0; i < 5; i++) {
        cap.open(i, cv::CAP_V4L2);  // Try different indices
        if (cap.isOpened()) break;
    }
    
    if (!cap.isOpened()) {
        cap.open(-1);  // Let OpenCV choose backend
    }
    
    if (!cap.isOpened()) {
        std::cerr << "ERROR: Failed to open camera after multiple attempts" << std::endl;
        return false;
    }
    
    // Set properties with fallbacks
    if (!cap.set(cv::CAP_PROP_FRAME_WIDTH, width)) {
        std::cerr << "WARNING: Failed to set width" << std::endl;
    }
    if (!cap.set(cv::CAP_PROP_FRAME_HEIGHT, height)) {
        std::cerr << "WARNING: Failed to set height" << std::endl;
    }
    
    // Check actual settings
    double actual_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double actual_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    
    std::cout << "Camera initialized at " << actual_width << "x" << actual_height << std::endl;
    initialized = true;
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
