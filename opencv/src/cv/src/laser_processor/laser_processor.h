#ifndef LASER_PROCESSOR_H
#define LASER_PROCESSOR_H

#include <opencv2/opencv.hpp>
#include <vector>

/**
 * @brief 膨胀腐蚀处理
 * @param result 输入图像
 * @return 处理后的图像
 */
cv::Mat& colorProcessAndDilateErode(cv::Mat& result);

/**
 * @brief 获取矩形，并标注
 * @param src 原始图像
 * @return 矩形的四个顶点
 */
std::vector<cv::Point> getRectAndMark(cv::Mat& src);

/**
 * @brief 处理轮廓并绘制边框
 * @param result 二值化图像
 * @param display 要显示边框的图像
 * @return 处理后的轮廓集合
 */
std::vector<std::vector<cv::Point>> processContours(const cv::Mat& result, cv::Mat& display);

/**
 * @brief 用于存储内外轮廓的结果
 */
struct ContourResult {
    std::vector<std::vector<cv::Point>> outer_contours; // 外轮廓
    std::vector<std::vector<cv::Point>> inner_contours; // 内轮廓
};

/**
 * @brief 查找并区分内外轮廓
 * @param src 输入图像
 * @return 包含内外轮廓的 ContourResult 对象
 */
ContourResult findInnerAndOuterRects(cv::Mat& src);

#endif // LASER_PROCESSOR_H 