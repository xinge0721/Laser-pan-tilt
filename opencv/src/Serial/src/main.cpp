#include <ros/ros.h>
#include <string>
#include "./Serial/Serial.h"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>  // 添加Int32消息类型的头文件
#include <std_srvs/Trigger.h>
#include "./HTS221/HTS221.h"
#include "Serial/date.h"
#include <iostream>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>
#include <stdlib.h> // 添加标准库头文件

// 信号处理函数
void signalHandler(int sig)
{
    ROS_INFO("捕获到Ctrl+C信号，正常退出程序...");
    // 使用ros::shutdown()代替exit(0)
    ros::shutdown();
}

HTS221 x(1); // 创建一个静态的HTS221对象，用于控制X轴
HTS221 y(2); // 创建一个静态的HTS221对象，用于控制Y轴

// 当前舵机状态
int current_x_angle = 600; // 初始X轴角度值
int current_y_angle = 500; // 初始Y轴角度值
const int FIXED_SPEED = 500;      // 固定舵机速度为500

// X轴角度滑块回调函数
void xAngleCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int angle = msg->data;
    // 确保角度在有效范围内
    if (angle < 0) angle = 0;
    if (angle > 1000) angle = 1000;
    
    // 更新当前角度值
    current_x_angle = angle;
    
    ROS_INFO("X轴角度设置为: %d", angle);
    
    // 直接调用turn函数设置舵机角度
    x.turn(angle, FIXED_SPEED);
}

// Y轴角度滑块回调函数
void yAngleCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int angle = msg->data;
    // 确保角度在有效范围内
    if (angle < 0) angle = 0;
    if (angle > 1000) angle = 1000;
    
    // 更新当前角度值
    current_y_angle = angle;
    
    ROS_INFO("Y轴角度设置为: %d", angle);
    
    // 直接调用turn函数设置舵机角度
    y.turn(angle, FIXED_SPEED);
}

/**
 * @brief 处理服务返回的角度数据
 * @param data 服务返回的字符串数据，格式为"x,y"
 */
void handleAngleData(const Serial::date& date, int& current_x_angle, int& current_y_angle)
{
    int x_angle = date.response.x;
    int y_angle = date.response.y;
 
    ROS_INFO("接收到角度数据: x=%d, y=%d", x_angle, y_angle);
    
    // 更新当前角度变量
    current_x_angle = x_angle;
    current_y_angle = y_angle;
    
    // 使用固定速度
    x.turn(x_angle, FIXED_SPEED);
    y.turn(y_angle, FIXED_SPEED);
}

int main(int argc, char *argv[])
{
    // 在ROS初始化前注册信号处理函数
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // 执行 ros 节点初始化
    ros::init(argc, argv, "HTS20L", ros::init_options::NoSigintHandler);

    // 创建 ros 节点句柄(非必须)
    ros::NodeHandle n;

    // 初始化舵机位置 - 直接使用精确角度值
    x.turn(current_x_angle, FIXED_SPEED);
    y.turn(current_y_angle, FIXED_SPEED);
    
    // 创建订阅者，订阅角度控制话题
    ros::Subscriber x_angle_sub = n.subscribe("/x_angle_slider", 1, xAngleCallback);
    ros::Subscriber y_angle_sub = n.subscribe("/y_angle_slider", 1, yAngleCallback);
    
    // 设置服务请求频率为10Hz
    ros::Rate rate(10);
    
    // 输出使用说明
    ROS_INFO("舵机精确角度控制已启动，固定速度为: %d", FIXED_SPEED);
    ROS_INFO("角度范围: 0-1000 (对应舵机角度0-240°)");
    ROS_INFO("X轴角度控制: /x_angle_slider (0-1000)");
    ROS_INFO("Y轴角度控制: /y_angle_slider (0-1000)");
    
    Serial::date mood;
    mood.request.mood = 1;  // 默认值设为1
    
    while(ros::ok())
    {
        // 处理回调函数
        ros::spinOnce();
        
        // 按照设定频率休眠
        rate.sleep();
    }

    return 0;
}