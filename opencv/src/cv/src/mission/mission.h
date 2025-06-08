#ifndef MISSION_H
#define MISSION_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include "./../laser_processor/laser_processor.h"
#include "./../hsv_threshold/hsv_threshold.h"
#include <vector>
// 红色阈值
extern HsvThreshold hsvThresh;

// 采用统一的裁剪比例
#define CROP_RATIO 0.78
#define X_RATIO 0.75

 cv::Mat cropCenterRegion(const cv::Mat& src, double crop_ratio = CROP_RATIO, double x_ratio = X_RATIO);

typedef struct Mission
{
    
    // 校准函数
    // 传入当前图像（激光的位置）
    // 返回屏幕坐标
    std::vector<cv::Point> calibration(HsvThreshold& hsvThresh,cv::VideoCapture& cap);

    // 第一题
    geometry_msgs::Point one(cv::VideoCapture& cap, ros::Publisher angle_pub, std::vector<cv::Point>& points);
    // 第二题
    void two(cv::VideoCapture& cap, ros::Publisher angle_pub, std::vector<cv::Point>& points);
    // 第三题
    void three(cv::VideoCapture& cap,ros::Publisher angle_pub);
    // 第四题
    void four(cv::VideoCapture& cap,ros::Publisher angle_pub);
    // 第五题
    void five(cv::VideoCapture& cap,ros::Publisher angle_pub);
    // 第六题
    void six(cv::VideoCapture& cap,ros::Publisher angle_pub);
    // 第七题
    void seven(cv::VideoCapture& cap,ros::Publisher angle_pub);
    // 第八题
    void eight(cv::VideoCapture& cap,ros::Publisher angle_pub);
}Mission;


#endif
