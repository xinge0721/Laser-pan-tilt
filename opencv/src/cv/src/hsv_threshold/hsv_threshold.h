#ifndef HSV_THRESHOLD_H
#define HSV_THRESHOLD_H

#include <opencv2/opencv.hpp>

// HSV阈值结构体
struct HsvThreshold {
    int H_min = 0, H_max = 179;
    int S_min = 0, S_max = 255;
    int V_min = 0, V_max = 255;
};

// 绿色预设值
extern HsvThreshold GREEN_HSV;

// 对图像进行HSV阈值处理
cv::Mat thresholdHsv(const cv::Mat& src, const HsvThreshold& thresh);

// 创建HSV阈值滑块
void createHsvSliders(const std::string& window_name, HsvThreshold& thresh);

#endif // HSV_THRESHOLD_H 