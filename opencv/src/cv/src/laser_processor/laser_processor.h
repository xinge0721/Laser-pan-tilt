#ifndef LASER_PROCESSOR_H
#define LASER_PROCESSOR_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <utility>

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
 * @brief 识别内外框, 返回包含内外轮廓的向量
 * @param src 二值化图像
 * @return 一个包含内外矩形顶点坐标对的pair。pair.first是外矩形，pair.second是内矩形。
 */
std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> findInnerAndOuterContours(const cv::Mat& src);

#endif // LASER_PROCESSOR_H 