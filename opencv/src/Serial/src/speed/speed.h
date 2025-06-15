#ifndef SPEED_H
#define SPEED_H

#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>

// 开环电机模式控制
// 参数一：距离（像素点）
// 参数二：舵机ID（1=X轴，2=Y轴）
int16_t setSpeed(int juli, int servo_id);



#endif