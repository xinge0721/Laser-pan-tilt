#include <ros/ros.h>
#include <string>
#include "./Serial/Serial.h"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h> // 包含Point消息类型
#include <iostream>
#include <signal.h>
#include <stdlib.h>

// 全局串口对象
myserial globalSerial;

// 信号处理函数
void signalHandler(int sig)
{
    ROS_INFO("捕获到Ctrl+C信号，正常退出程序...");
    // 关闭串口
    globalSerial.close();
    ros::shutdown();
}

/**
 * @brief 处理来自GUI节点的增量数据
 * @param msg 包含x和y增量步数的消息
 */
void angleDataCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    int x_increment = static_cast<int>(msg->x);
    int y_increment = static_cast<int>(msg->y);
 
    ROS_INFO("接收到增量数据: x=%d, y=%d", x_increment, y_increment);
    
    // 调用sendPoint通过串口发送增量数据
    if (!globalSerial.sendPoint(x_increment, y_increment)) {
        ROS_ERROR("通过sendPoint发送数据失败！");
    }
}

int main(int argc, char *argv[])
{
    // 在ROS初始化前注册信号处理函数
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // 执行 ros 节点初始化
    ros::init(argc, argv, "digital_servo_controller", ros::init_options::NoSigintHandler);

    // 创建 ros 节点句柄
    ros::NodeHandle n;
    
    // 初始化全局串口
    if (!globalSerial.init("/dev/ttyUSB0", 115200)) {
        ROS_ERROR("全局串口初始化失败!");
        return -1;
    }
    
    // 创建订阅者，订阅GUI发布的增量数据
    ros::Subscriber angle_data_sub = n.subscribe("angle_data", 10, angleDataCallback);
    
    // 提示信息
    ROS_INFO("数字舵机控制器已启动，等待来自GUI的增量数据...");
    
    // 使用ros::spin()进入循环，等待回调函数被触发
    // 当接收到消息时，angleDataCallback会被自动调用
    ros::spin();
    
    return 0;
}