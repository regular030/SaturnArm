#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

class Camera {
public:
    Camera(int width, int height);
    bool init();
    bool capture_frame(std::vector<uchar>& buffer);
    
private:
    int width;
    int height;
    cv::VideoCapture cap;
};
