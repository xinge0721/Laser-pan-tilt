#include <ros/ros.h>
#include <string>
#include "./Serial/Serial.h"
#include <std_msgs/String.h>
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
/**
 * @brief 处理服务返回的角度数据
 * @param data 服务返回的字符串数据，格式为"x,y"
 */
void handleAngleData(const Serial::date& date, int& current_x_angle, int& current_y_angle)
{
    static int speed = 50; // 舵机速度改为50


    int x_angle = date.response.x;
    int y_angle = date.response.y;
 
    ROS_INFO("接收到角度数据: x=%d, y=%d", x_angle, y_angle);
    
    // 更新当前角度变量
    current_x_angle = x_angle;
    current_y_angle = y_angle;
    
    // 参数一 角度 参数二 速度
    x.turn(x_angle,speed);
    
    // 安全距离限制已取消，直接转动Y轴
    y.turn(y_angle,speed);
}

// 非阻塞方式检查标准输入是否有数据可读
bool kbhit()
{
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0;
}

int main(int argc, char *argv[])
{
    // 在ROS初始化前注册信号处理函数
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    //执行 ros 节点初始化
    ros::init(argc, argv, "HTS20L", ros::init_options::NoSigintHandler);

    //创建 ros 节点句柄(非必须)
    ros::NodeHandle n;

    // 初始化舵机位置
    int current_x_angle = 600;
    int current_y_angle = 500;
    x.turn(current_x_angle,50); // 舵机速度改为50
    y.turn(current_y_angle,50); // 舵机速度改为50
    // // 创建角度服务客户端，连接到camera节点提供的angle_service服务
    // ros::ServiceClient angle_client = n.serviceClient<Serial::date>("angle_service");

    // ros::service::waitForService("angle_service");
    // 设置服务请求频率为10Hz
    ros::Rate rate(10);
    
    // 循环请求角度服务
    Serial::date mood;
    mood.request.mood = 1;  // 默认值设为1
    
    std::cout << "按键控制说明：" << std::endl;
    std::cout << "按1: 模式1" << std::endl;
    std::cout << "按2: 模式2" << std::endl;
    std::cout << "按3: 模式3" << std::endl;
    std::cout << "按a: X轴舵机角度+100" << std::endl;
    std::cout << "按s: X轴舵机角度-100" << std::endl;
    std::cout << "按d: Y轴舵机角度+100" << std::endl;
    std::cout << "按f: Y轴舵机角度-100" << std::endl;
    
    while(ros::ok())
    {
        // 非阻塞方式检查是否有新的按键输入
        if(kbhit()) {
            char key;
            std::cin >> key;
            
            // 根据按键设置mood值
            if(key == '1' || key == '2' || key == '3') {
                mood.request.mood = key - '0';  // 将字符转换为数字
                ROS_INFO("设置模式为%d", mood.request.mood);
            } else if(key == 'a') {
                // X舵机角度+100
                current_x_angle += 100;
                x.turn(current_x_angle, 50); // 舵机速度改为50
                ROS_INFO("X轴舵机角度增加100，当前角度: %d", current_x_angle);
            } else if(key == 's') {
                // X舵机角度-100
                current_x_angle -= 100;
                x.turn(current_x_angle, 50); // 舵机速度改为50
                ROS_INFO("X轴舵机角度减少100，当前角度: %d", current_x_angle);
            } else if(key == 'd') {
                // Y舵机角度+100
                current_y_angle += 100;
                y.turn(current_y_angle, 50); // 舵机速度改为50
                ROS_INFO("Y轴舵机角度增加100，当前角度: %d", current_y_angle);
            } else if(key == 'f') {
                // Y舵机角度-100
                current_y_angle -= 100;
                y.turn(current_y_angle, 50); // 舵机速度改为50
                ROS_INFO("Y轴舵机角度减少100，当前角度: %d", current_y_angle);
            } else {
                ROS_INFO("无效输入，请输入1、2、3、a、s、d或f");
            }
        }

        bool flag;
        // bool flag = angle_client.call(mood);
        // 调用服务并处理结果11
        if(flag)
        {
            // 处理成功获取的角度数据
            handleAngleData(mood, current_x_angle, current_y_angle);
        }
        else
        {
            // 服务调用失败
            ROS_ERROR("调用角度服务失败");
        }
        // 处理回调
        ros::spinOnce();
        // 按照设定频率休眠
        rate.sleep();
    }

    return 0;
}