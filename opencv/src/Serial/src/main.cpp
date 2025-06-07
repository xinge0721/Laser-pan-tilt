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
#include <thread>   // 添加线程支持
#include <atomic>   // 添加原子变量支持
#include <chrono>   // 添加时间相关头文件

// 全局串口对象
myserial globalSerial;

// 线程控制变量
std::atomic<bool> threadRunning(false);
std::thread serialThread;

// 全局角度限制变量
int X_MIN_ANGLE = 613;  // X轴最小角度
int X_MAX_ANGLE = 823;  // X轴最大角度
int Y_MIN_ANGLE = 562;  // Y轴最小角度
int Y_MAX_ANGLE = 750;  // Y轴最大角度

// 全局目标角度变量 - 由滑块控制更新
int target_angle_x = 400;
int target_angle_y = 637; // 添加Y轴目标角度

// 当前舵机状态 
int current_x_angle = 1000;      // 初始X轴角度值
int current_y_angle = 1000;      // 初始Y轴角度值
const int FIXED_SPEED = 50;      // 固定舵机速度为100

// 信号处理函数
void signalHandler(int sig)
{
    ROS_INFO("捕获到Ctrl+C信号，正常退出程序...");
    
    // 停止接收线程
    threadRunning = false;
    if (serialThread.joinable()) {
        serialThread.join();
    }
    
    // 使用ros::shutdown()代替exit(0)
    ros::shutdown();
}

HTS221 x(1); // 创建一个X轴舵机对象，ID=1
HTS221 y(2); // 创建一个Y轴舵机对象，ID=2

// 简单的串口接收线程函数 - 类似STM32的串口中断处理
void serialReceiveThread() {
    uint8_t buffer[1]; // 单字节缓冲区
    
 
    
    std::cout << "串口接收线程已启动" << std::endl;
    
    while (threadRunning) {
      
        
        // 尝试读取一个字节
        int bytesRead = globalSerial.readBuffer(buffer, 1);
        
        // 如果成功读取到数据
        if (bytesRead > 0) {
            // 将字节发送给舵机对象处理
            x.returnData(buffer[0]);
            y.returnData(buffer[0]);
        } else {
            // 如果没有数据，短暂休眠以降低CPU使用率
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    std::cout << "串口接收线程已停止" << std::endl;
}

// X轴角度滑块回调函数
void xAngleCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int angle = msg->data;
    // 确保角度在有效范围内
    if (angle < X_MIN_ANGLE) angle = X_MIN_ANGLE;
    if (angle > X_MAX_ANGLE) angle = X_MAX_ANGLE;
    
    // 只更新目标角度，不直接控制舵机
    target_angle_x = angle;
    
    // 打印接收到的角度值
    std::cout << "接收X轴目标角度: " << angle << std::endl;
}

// Y轴角度滑块回调函数
void yAngleCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int angle = msg->data;
    // 确保角度在有效范围内
    if (angle < Y_MIN_ANGLE) angle = Y_MIN_ANGLE;
    if (angle > Y_MAX_ANGLE) angle = Y_MAX_ANGLE;
    
    // 只更新目标角度，不直接控制舵机
    target_angle_y = angle;
    
    // 打印接收到的角度值
    std::cout << "接收Y轴目标角度: " << angle << std::endl;
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
    
    // 初始化全局串口
    if (!globalSerial.init("/dev/ttyUSB0", 115200)) {
        ROS_ERROR("全局串口初始化失败!");
        return -1;
    }
    
    // 启动串口接收线程
    threadRunning = true;
    serialThread = std::thread(serialReceiveThread);

    // 注意：不再创建局部舵机对象，而是使用全局对象x和y
    
    // 创建订阅者，订阅角度控制话题
    ros::Subscriber x_angle_sub = n.subscribe("/x_angle_slider", 1, xAngleCallback);
    ros::Subscriber y_angle_sub = n.subscribe("/y_angle_slider", 1, yAngleCallback);
    
    // 创建订阅者，订阅相机发布的角度数据
    ros::Subscriber angle_data_sub = n.subscribe("angle_data", 10, angleDataCallback);
    
    // 创建订阅者，订阅相机发布的模式
    ros::Subscriber mode_sub = n.subscribe("camera_mode", 10, modeCallback);
    
    // 设置循环频率为100Hz（10ms一次）
    ros::Rate rate(100);
    
    // 简单提示
    std::cout << "控制模式启动 - 串行控制X和Y轴舵机" << std::endl;
    std::cout << "X轴初始目标角度: " << target_angle_x << std::endl;
    std::cout << "Y轴初始目标角度: " << target_angle_y << std::endl;
    
    // 控制模式标志和计数器
    bool control_mode = false;
    int cycle_count = 0;
    
    while(ros::ok())
    {
        // 轮流处理，防止转动的同时请求数据
        control_mode = !control_mode;
        
        if (control_mode) {
            // 发送读取角度请求（每3个循环交替读取X和Y）
            // 读取X轴角度
            x.readAngle();
            x.return_flag = false;
            while(!x.return_flag); // 没拿到数据就卡在着,阻塞
        
            // 读取Y轴角度
            y.readAngle();
            y.return_flag = false;
            while(!y.return_flag); // 没拿到数据就卡在着,阻塞
        }
        else {
            // 控制X和Y轴舵机（在这里集中控制，而不是回调函数中）
                // 控制X轴舵机
                x.turn(target_angle_x, FIXED_SPEED);
                std::cout << "控制X轴: " << target_angle_x << std::endl;
                
                // 控制Y轴舵机
                y.turn(target_angle_y, FIXED_SPEED);
                std::cout << "控制Y轴: " << target_angle_y << std::endl;

        }
        
        rate.sleep();
        ros::spinOnce();
    }
    
    // 程序结束前清理资源
    threadRunning = false;
    if (serialThread.joinable()) {
        serialThread.join();
    }
    
    // 关闭串口
    globalSerial.close();

    return 0;
}