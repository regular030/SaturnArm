#pragma once
#include <opencv2/opencv.hpp> // not used atm
#include <vector>
#include <string>

class Camera {
public:
    Camera(int width, int height);
    bool init();
    bool capture_frame(std::vector<uchar>& buffer);
    bool is_initialized() const { return initialized; }
    
private:
    int width;
    int height;
    bool initialized = false;
    cv::VideoCapture cap;
};
