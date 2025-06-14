//#include <ros/ros.h>
//#include <std_msgs/Int32.h>
//#include <geometry_msgs/Point.h>
#include <signal.h>
#include <pigpio.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>

// 舵机控制引脚定义
#define X_SERVO_PIN 18  // 树莓派GPIO18，PWM控制X轴舵机
#define Y_SERVO_PIN 19  // 树莓派GPIO19，PWM控制Y轴舵机

// 舵机PWM参数（标准舵机通常是500-2500μs对应0-180度）
#define MIN_PULSE_WIDTH 500   // 最小脉冲宽度(0度位置)，单位为μs
#define MAX_PULSE_WIDTH 2500  // 最大脉冲宽度(180度位置)，单位为μs
#define DEFAULT_PULSE_WIDTH 1500  // 中间位置(90度)脉冲宽度

// 舵机角度范围
#define MIN_ANGLE 0
#define MAX_ANGLE 180

// 全局目标角度变量
int target_angle_x = 90;  // X轴目标角度（初始为中间位置）
int target_angle_y = 90;  // Y轴目标角度（初始为中间位置）

// 当前接收到的摄像头角度数据
int current_x_angle = 0;
int current_y_angle = 0;

// 线程控制变量
std::atomic<bool> threadRunning(false);
std::thread servoControlThread;

// 信号处理函数
void signalHandler(int sig)
{
    std::cout << "捕获到Ctrl+C信号，正常退出程序..." << std::endl;
    
    // 停止控制线程
    threadRunning = false;
    if (servoControlThread.joinable()) {
        servoControlThread.join();
    }
    
    // 关闭PWM并终止pigpio
    gpioServo(X_SERVO_PIN, 0);
    gpioServo(Y_SERVO_PIN, 0);
    gpioTerminate();
    
    exit(0);
}

// 将角度转换为脉冲宽度
int angleToPulse(int angle)
{
    // 限制角度范围
    if (angle < MIN_ANGLE) angle = MIN_ANGLE;
    if (angle > MAX_ANGLE) angle = MAX_ANGLE;
    
    // 线性映射：角度 -> 脉冲宽度
    return MIN_PULSE_WIDTH + (angle * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / MAX_ANGLE);
}

// 以下是ROS回调函数，暂时注释掉
/*
// X轴角度回调函数
void xAngleCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int angle = msg->data;
    
    // 确保角度在有效范围内
    if (angle < MIN_ANGLE) angle = MIN_ANGLE;
    if (angle > MAX_ANGLE) angle = MAX_ANGLE;
    
    // 更新目标角度
    target_angle_x = angle;
    
    std::cout << "接收X轴目标角度: " << angle << "度" << std::endl;
}

// Y轴角度回调函数
void yAngleCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int angle = msg->data;
    
    // 确保角度在有效范围内
    if (angle < MIN_ANGLE) angle = MIN_ANGLE;
    if (angle > MAX_ANGLE) angle = MAX_ANGLE;
    
    // 更新目标角度
    target_angle_y = angle;
    
    std::cout << "接收Y轴目标角度: " << angle << "度" << std::endl;
}

// 处理来自相机节点的角度数据
void angleDataCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    int x_angle = static_cast<int>(msg->x);
    int y_angle = static_cast<int>(msg->y);

    // 更新当前角度变量
    current_x_angle = x_angle;
    current_y_angle = y_angle;
    
    std::cout << "接收到角度数据: x=" << x_angle << ", y=" << y_angle << std::endl;
}

// 处理相机节点发来的模式
void modeCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int mode = msg->data;
    std::cout << "接收到相机模式: " << mode << std::endl;
    // 可以根据模式做不同处理
}
*/

// 舵机控制线程函数
void servoControlLoop()
{
    int current_x_pulse = DEFAULT_PULSE_WIDTH;
    int current_y_pulse = DEFAULT_PULSE_WIDTH;
    int target_x_pulse, target_y_pulse;
    
    // 设置舵机初始位置
    gpioServo(X_SERVO_PIN, current_x_pulse);
    gpioServo(Y_SERVO_PIN, current_y_pulse);
    
    std::cout << "舵机控制线程已启动" << std::endl;
    
    while (threadRunning)
    {
        // 计算目标脉冲宽度
        target_x_pulse = angleToPulse(target_angle_x);
        target_y_pulse = angleToPulse(target_angle_y);
        
        // 平滑过渡到目标位置
        if (current_x_pulse != target_x_pulse) {
            // 计算步进值（可调整步进大小控制速度）
            int step = (current_x_pulse < target_x_pulse) ? 10 : -10;
            current_x_pulse += step;
            
            // 检查是否已达到或超过目标
            if ((step > 0 && current_x_pulse > target_x_pulse) || 
                (step < 0 && current_x_pulse < target_x_pulse)) {
                current_x_pulse = target_x_pulse;
            }
            
            // 设置舵机位置
            gpioServo(X_SERVO_PIN, current_x_pulse);
        }
        
        // 添加小延迟，避免X轴和Y轴同时移动可能引起的电流过大
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // 对Y轴做同样的处理
        if (current_y_pulse != target_y_pulse) {
            int step = (current_y_pulse < target_y_pulse) ? 10 : -10;
            current_y_pulse += step;
            
            if ((step > 0 && current_y_pulse > target_y_pulse) || 
                (step < 0 && current_y_pulse < target_y_pulse)) {
                current_y_pulse = target_y_pulse;
            }
            
            gpioServo(Y_SERVO_PIN, current_y_pulse);
        }
        
        // 控制循环频率
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    std::cout << "舵机控制线程已停止" << std::endl;
}

// 简单测试函数 - 让舵机在不同角度间移动
void testServo()
{
    while(threadRunning) {
        std::cout << "设置舵机角度: X=45, Y=45" << std::endl;
        target_angle_x = 45;
        target_angle_y = 45;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        std::cout << "设置舵机角度: X=135, Y=45" << std::endl;
        target_angle_x = 135;
        target_angle_y = 45;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        std::cout << "设置舵机角度: X=135, Y=135" << std::endl;
        target_angle_x = 135;
        target_angle_y = 135;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        std::cout << "设置舵机角度: X=45, Y=135" << std::endl;
        target_angle_x = 45;
        target_angle_y = 135;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        std::cout << "设置舵机角度: X=90, Y=90 (中间位置)" << std::endl;
        target_angle_x = 90;
        target_angle_y = 90;
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

int main(int argc, char *argv[])
{
    // 注册信号处理函数
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // ROS相关初始化 - 已注释掉
    // ros::init(argc, argv, "digital_servo_node", ros::init_options::NoSigintHandler);
    // ros::NodeHandle n;
    
    // 初始化pigpio库
    if (gpioInitialise() < 0) {
        std::cerr << "无法初始化pigpio库!" << std::endl;
        return -1;
    }
    
    std::cout << "pigpio库初始化成功" << std::endl;
    
    // 设置GPIO引脚为输出模式
    gpioSetMode(X_SERVO_PIN, PI_OUTPUT);
    gpioSetMode(Y_SERVO_PIN, PI_OUTPUT);
    
    // 启动舵机控制线程
    threadRunning = true;
    servoControlThread = std::thread(servoControlLoop);

    // ROS订阅相关 - 已注释掉
    // ros::Subscriber x_angle_sub = n.subscribe("/x_angle_slider", 1, xAngleCallback);
    // ros::Subscriber y_angle_sub = n.subscribe("/y_angle_slider", 1, yAngleCallback);
    // ros::Subscriber angle_data_sub = n.subscribe("angle_data", 10, angleDataCallback);
    // ros::Subscriber mode_sub = n.subscribe("camera_mode", 10, modeCallback);
    
    // 简单提示
    std::cout << "舵机测试程序已启动" << std::endl;
    std::cout << "X轴初始目标角度: " << target_angle_x << "度" << std::endl;
    std::cout << "Y轴初始目标角度: " << target_angle_y << "度" << std::endl;
    std::cout << "开始测试舵机运动..." << std::endl;
    
    // 运行测试函数
    testServo();
    
    // ROS主循环 - 已注释掉
    // ros::spin();
    
    // 程序结束前清理资源
    threadRunning = false;
    if (servoControlThread.joinable()) {
        servoControlThread.join();
    }
    
    // 关闭PWM并终止pigpio
    gpioServo(X_SERVO_PIN, 0);
    gpioServo(Y_SERVO_PIN, 0);
    gpioTerminate();

    return 0;
} 