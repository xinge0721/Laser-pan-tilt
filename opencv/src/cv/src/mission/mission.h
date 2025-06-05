#ifndef MISSION_H
#define MISSION_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include "./../laser_processor/laser_processor.h"

typedef struct Mission
{
    // 前俩提走固定路线，所以不需要图片
    // 第一题
    void one(ros::Publisher angle_pub);
    // 第二题
    void two(ros::Publisher angle_pub);
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
