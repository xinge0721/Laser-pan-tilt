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
#include <math.h>
#include "./speed/speed.h"
using std::cout;
using std::endl;

// 创建AngleData实例用于角度转换
AngleData angleConverter;

// 全局串口对象
myserial globalSerial;

// 线程控制变量
std::atomic<bool> threadRunning(false);
std::thread serialThread;

// 全局角度限制变量 (现在使用角度值，而非数据值)
int X_MIN_ANGLE = 0;   // X轴最小角度
int X_MAX_ANGLE = 240; // X轴最大角度
int Y_MIN_ANGLE = 0;   // Y轴最小角度
int Y_MAX_ANGLE = 240; // Y轴最大角度

// 全局目标角度变量
// X轴和Y轴具体目标角度（角度值0-240度）
int target_angle_x = 168; // 相当于原始数据值约701
int target_angle_y = 156; // 相当于原始数据值约650

// 当前舵机状态 
int current_x_angle = 240;      // 初始X轴角度值
int current_y_angle = 240;      // 初始Y轴角度值
const int FIXED_SPEED = 50;     // 固定舵机速度为50

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
    
    cout << "串口接收线程已启动" << endl;
    
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
    
    cout << "串口接收线程已停止" << endl;
}

// X轴角度滑块回调函数（接收到的是角度值0-240度）
void xAngleCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int angle = msg->data;
    // 确保角度在有效范围内
    if (angle < X_MIN_ANGLE) angle = X_MIN_ANGLE;
    if (angle > X_MAX_ANGLE) angle = X_MAX_ANGLE;
    
    // 更新目标角度（保存的是角度值）
    target_angle_x = angle;
    
    // 打印接收到的角度值
    std::cout << "接收X轴目标角度: " << angle << "度 (数据值: " << angleConverter.angleToData(angle) << ")" << std::endl;
}

// Y轴角度滑块回调函数（接收到的是角度值0-240度）
void yAngleCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int angle = msg->data;
    // 确保角度在有效范围内
    if (angle < Y_MIN_ANGLE) angle = Y_MIN_ANGLE;
    if (angle > Y_MAX_ANGLE) angle = Y_MAX_ANGLE;
    
    // 更新目标角度（保存的是角度值）
    target_angle_y = angle;
    
    // 打印接收到的角度值
    std::cout << "接收Y轴目标角度: " << angle << "度 (数据值: " << angleConverter.angleToData(angle) << ")" << std::endl;
}

/**
 * @brief 处理来自相机节点的角度数据
 * @param msg 角度数据消息
 */
void angleDataCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    int x_angle = static_cast<int>(msg->x);
    int y_angle = static_cast<int>(msg->y);
 
    // // 限制角度范围
    // if (x_angle < X_MIN_ANGLE) x_angle = X_MIN_ANGLE;
    // if (x_angle > X_MAX_ANGLE) x_angle = X_MAX_ANGLE;
    // if (y_angle < Y_MIN_ANGLE) y_angle = Y_MIN_ANGLE;
    // // if (y_angle > Y_MAX_ANGLE) y_angle = Y_MAX_ANGLE;

    // ROS_INFO("接收到角度数据: x=%d, y=%d", x_angle, y_angle);
    
    // 更新当前角度变量
    current_x_angle = x_angle;
    current_y_angle = y_angle;
    
    // // 转换角度为数据值，然后控制舵机
    // int x_data = angleConverter.angleToData(x_angle);
    // int y_data = angleConverter.angleToData(y_angle);
    ROS_INFO("接收到角度数据: x=%d, y=%d", x_angle, y_angle);
    // // 使用固定速度
    // x.turn(x_data, FIXED_SPEED);
    // y.turn(y_data, FIXED_SPEED);
    setSpeed(x,x_angle);
    setSpeed(y,y_angle);
}

/**
 * @brief 处理相机节点发来的模式
 */
void modeCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int mode = msg->data;
    // ROS_INFO("接收到相机模式: %d", mode);
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
    std::cout << "X轴初始目标角度: " << target_angle_x << "度 (数据值: " << angleConverter.angleToData(target_angle_x) << ")" << std::endl;
    std::cout << "Y轴初始目标角度: " << target_angle_y << "度 (数据值: " << angleConverter.angleToData(target_angle_y) << ")" << std::endl;
    
    // 控制模式标志和计数器
    bool control_mode = false;
    
    while(ros::ok())
    {
        // 轮流处理，防止转动的同时请求数据
        // 读取数据的时候，不能转动角度，否则会冲突
        // 因为舵机的信号线只有一条，而且相同的指令，会被“覆盖”
        control_mode = !control_mode;
        
        if (control_mode) {

            // 发送读取角度请求
            // 读取X轴角度
            x.readAngle();
            x.return_flag = false;
            
            // 添加超时机制，最多等待50ms
            auto start_time = std::chrono::steady_clock::now();
            while(!x.return_flag) {
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
                if (elapsed > 50) {
                    cout << "X轴读取超时" << endl;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            // 读取Y轴角度
            // y.readAngle();
            // y.return_flag = false;
            
            // // 添加超时机制，最多等待50ms
            // start_time = std::chrono::steady_clock::now();
            // while(!y.return_flag) {
            //     auto now = std::chrono::steady_clock::now();
            //     auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
            //     if (elapsed > 50) {
            //         cout << "Y轴读取超时" << endl;
            //         break;
            //     }
            //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
            // }
        }
        else {
            // 控制X和Y轴舵机（在这里集中控制，而不是回调函数中）
            // 将角度值转换为数据值，然后控制X轴舵机
            int x_data = angleConverter.angleToData(target_angle_x);
            x.turn(x_data, FIXED_SPEED);
            // cout << "控制X轴: " << target_angle_x << "度 (数据值: " << x_data << ")" << endl;
            
            // 等待一小段时间，让命令完成发送，避免命令冲突
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            
            // 将角度值转换为数据值，然后控制Y轴舵机
            int y_data = angleConverter.angleToData(target_angle_y);
            y.turn(y_data, FIXED_SPEED);
            // cout << "控制Y轴: " << target_angle_y << "度 (数据值: " << y_data << ")" << endl;
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