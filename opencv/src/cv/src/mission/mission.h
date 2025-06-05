#ifndef MISSION_H
#define MISSION_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include "./../laser_processor/laser_processor.h"
#include "./../hsv_threshold/hsv_threshold.h"

typedef struct Mission
{
    
    // 校准函数
    // 传入当前图像（激光的位置）
    // 返回屏幕坐标
    std::vector<cv::Point> calibration(cv::Mat frame,HsvThreshold& hsvThresh,cv::VideoCapture& cap);

    // 第一题
    void one(cv::Mat frame,ros::Publisher angle_pub);
    // 第二题
    void two(cv::Mat frame,ros::Publisher angle_pub);
    // 第三题
    void three(cv::Mat frame,ros::Publisher angle_pub);
    // 第四题
    void four(cv::Mat frame,ros::Publisher angle_pub);
    // 第五题
    void five(cv::Mat frame,ros::Publisher angle_pub);
    // 第六题
    void six(cv::Mat frame,ros::Publisher angle_pub);
    // 第七题
    void seven(cv::Mat frame,ros::Publisher angle_pub);
    // 第八题
    void eight(cv::Mat frame,ros::Publisher angle_pub);
}Mission;


#endif
