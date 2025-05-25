#include "hsv_threshold.h"

// 绿色预设值
HsvThreshold GREEN_HSV = {
    35,  // H_min - 色调最小值
    85,  // H_max - 色调最大值
    14,  // S_min - 饱和度最小值
    255, // S_max - 饱和度最大值
    40,  // V_min - 亮度最小值
    255  // V_max - 亮度最大值
};

// 对图像进行HSV阈值处理
cv::Mat thresholdHsv(const cv::Mat& src, const HsvThreshold& thresh)
{
    // 转换为HSV颜色空间
    cv::Mat hsv;
    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
    
    // 定义HSV阈值范围
    cv::Scalar lower(thresh.H_min, thresh.S_min, thresh.V_min);
    cv::Scalar upper(thresh.H_max, thresh.S_max, thresh.V_max);
    
    // 应用阈值
    cv::Mat result;
    cv::inRange(hsv, lower, upper, result);
    
    return result;
}

// 创建HSV阈值滑块
void createHsvSliders(const std::string& window_name, HsvThreshold& thresh)
{
    // 创建一个窗口
    cv::namedWindow(window_name);
    
    // H通道滑块 (0-179)
    cv::createTrackbar("H min", window_name, &thresh.H_min, 179, nullptr);
    cv::createTrackbar("H max", window_name, &thresh.H_max, 179, nullptr);
    
    // S通道滑块 (0-255)
    cv::createTrackbar("S min", window_name, &thresh.S_min, 255, nullptr);
    cv::createTrackbar("S max", window_name, &thresh.S_max, 255, nullptr);
    
    // V通道滑块 (0-255)
    cv::createTrackbar("V min", window_name, &thresh.V_min, 255, nullptr);
    cv::createTrackbar("V max", window_name, &thresh.V_max, 255, nullptr);
}

/**
 * @brief 初始化HSV阈值处理环境
 * @return HSV阈值结构体
 */
HsvThreshold initHsvThreshold()
{
    // 创建HSV阈值调节滑块
    createHsvSliders("HSV阈值控制", GREEN_HSV);
    
    return GREEN_HSV;
}

/**
 * @brief 处理图像并显示HSV阈值结果
 * @param image 输入图像
 * @param hsvThresh HSV阈值参数
 * @return 处理后的二值图像
 */
cv::Mat processAndShowHsvThreshold(const cv::Mat& image, HsvThreshold& hsvThresh)
{
    // 使用HSV阈值处理
    cv::Mat threshold_result = thresholdHsv(image, hsvThresh);
    
    // 显示结果
    cv::imshow("HSV阈值处理", threshold_result);
    
    return threshold_result;
} 