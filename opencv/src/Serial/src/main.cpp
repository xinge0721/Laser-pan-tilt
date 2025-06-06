#include <ros/ros.h>
#include <string>
#include "./Serial/Serial.h"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>  // 添加Int32消息类型的头文件
#include <geometry_msgs/Point.h> // 添加Point消息类型
#include "./HTS221/HTS221.h"
#include <iostream>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>
#include <stdlib.h> // 添加标准库头文件

// 全局角度限制变量
int X_MIN_ANGLE = 613;  // X轴最小角度
int X_MAX_ANGLE = 823;  // X轴最大角度
int Y_MIN_ANGLE = 562;  // Y轴最小角度
int Y_MAX_ANGLE = 750;  // Y轴最大角度

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
int current_x_angle = 1000;      // 初始X轴角度值
int current_y_angle = 1000;      // 初始Y轴角度值
const int FIXED_SPEED = 50;      // 固定舵机速度为100

// X轴角度滑块回调函数
void xAngleCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int angle = msg->data;
    // 确保角度在有效范围内
    if (angle < X_MIN_ANGLE) angle = X_MIN_ANGLE;
    if (angle > X_MAX_ANGLE) angle = X_MAX_ANGLE;
    
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
    if (angle < Y_MIN_ANGLE) angle = Y_MIN_ANGLE;
    if (angle > Y_MAX_ANGLE) angle = Y_MAX_ANGLE;
    
    // 更新当前角度值
    current_y_angle = angle;
    
    ROS_INFO("Y轴角度设置为: %d", angle);
    
    // 直接调用turn函数设置舵机角度
    y.turn(angle, FIXED_SPEED);
}

/**
 * @brief 处理来自相机节点的角度数据
 * @param msg 角度数据消息
 */
void angleDataCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    int x_angle = static_cast<int>(msg->x);
    int y_angle = static_cast<int>(msg->y);
 
    // 限制角度范围
    if (x_angle < X_MIN_ANGLE) x_angle = X_MIN_ANGLE;
    if (x_angle > X_MAX_ANGLE) x_angle = X_MAX_ANGLE;
    if (y_angle < Y_MIN_ANGLE) y_angle = Y_MIN_ANGLE;
    if (y_angle > Y_MAX_ANGLE) y_angle = Y_MAX_ANGLE;

    ROS_INFO("接收到角度数据: x=%d, y=%d", x_angle, y_angle);
    
    // 更新当前角度变量
    current_x_angle = x_angle;
    current_y_angle = y_angle;
    
    // 使用固定速度
    x.turn(x_angle, FIXED_SPEED);
    usleep(50000); // 暂停100毫秒
    y.turn(y_angle, FIXED_SPEED);
}

/**
 * @brief 处理相机节点发来的模式
 */
void modeCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int mode = msg->data;
    ROS_INFO("接收到相机模式: %d", mode);
    // 可以根据模式做不同处理
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
    
    // 启动X轴和Y轴舵机的接收线程
    x.startReceiveThread();
    y.startReceiveThread();
    
    // 创建订阅者，订阅角度控制话题
    ros::Subscriber x_angle_sub = n.subscribe("/x_angle_slider", 1, xAngleCallback);
    ros::Subscriber y_angle_sub = n.subscribe("/y_angle_slider", 1, yAngleCallback);
    
    // 创建订阅者，订阅相机发布的角度数据
    ros::Subscriber angle_data_sub = n.subscribe("angle_data", 10, angleDataCallback);
    
    // 创建订阅者，订阅相机发布的模式
    ros::Subscriber mode_sub = n.subscribe("camera_mode", 10, modeCallback);
    
    // 设置循环频率为10Hz
    ros::Rate rate(10);
    
    // 输出使用说明
    ROS_INFO("舵机精确角度控制已启动，固定速度为: %d", FIXED_SPEED);
    ROS_INFO("X轴角度范围: %d-%d", X_MIN_ANGLE, X_MAX_ANGLE);
    ROS_INFO("Y轴角度范围: %d-%d", Y_MIN_ANGLE, Y_MAX_ANGLE);
    ROS_INFO("X轴角度控制: /x_angle_slider");
    ROS_INFO("Y轴角度控制: /y_angle_slider");
    ROS_INFO("从相机节点接收角度数据: angle_data");
    ROS_INFO("从相机节点接收模式数据: camera_mode");
    ROS_INFO("舵机接收线程已启动");
    
    while(ros::ok())
    {
        // 处理回调函数
        ros::spinOnce();
        
        // 按照设定频率休眠
        rate.sleep();
    }
    
    // 停止接收线程
    x.stopReceiveThread();
    y.stopReceiveThread();

    return 0;
}