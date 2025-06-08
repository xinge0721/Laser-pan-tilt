#include "speed.h"
#include "../HTS221/HTS221.h"


// 先尝试 开环电机控制
// 不写类

// 开环电机模式控制
// 参数一：舵机对象
// 参数二：距离（像素点）
// 速度范围：-1000-1000
// 特别注意当speed在-100~100之间时，根本不会转了
// 而零数据舵机不会响应，所以得给一个很小的数据，让舵机"停下"
// 特别注意，停止指令hts221.stop()，只有在位置控制模式有效，电机模式不论怎么发，舵机都不管你
void setSpeed(HTS221& hts221, int juli)
{
    int16_t speed = 0;
    // 限制 y 轴坐标范围，防止舵机卡死
    // 这里限制范围为 200-1000，因为舵机在 200-1600 之间才能正常工作
    if(hts221.ID == 2)
    {
        if(hts221.angle < 400 && juli < 0) {
            // 已达下限且还想减少，阻止
            hts221.setMotorMode(5);
            return;
        } 
        else if(hts221.angle > 1000 && juli > 0) {
            // 已达上限且还想增加，阻止
            hts221.setMotorMode(5);
            return;
        }
    }
    
    // 限制 x 轴坐标范围，防止舵机卡死
    // 这里限制范围为 500-1000，因为舵机在 200-1600 之间才能正常工作
    if(hts221.ID == 1)
    {
        if(hts221.angle < 400 && juli > 0) {
            // 已达下限且还想减少，阻止
            hts221.setMotorMode(5);
            return;
        }
        else if(hts221.angle > 1000 && juli < 0) {
            // 已达上限且还想增加，阻止
            hts221.setMotorMode(5);
            return;
        }
    }
    
    if(juli > 200)speed = 1000;
    else if(juli > 100)speed = 800;
    else if(juli > 50)speed = 600;
    else if(juli > 40)speed = 400;
    else if(juli > 20)speed = 200;
    else if(juli > 10)speed = 100;
    else if(juli < -200)speed = -1000;
    else if(juli < -100)speed = -800;
    else if(juli < -50)speed = -600;
    else if(juli < -40)speed = -400;
    else if(juli < -20)speed = -200;
    else if(juli < -10)speed = -100;
    else speed = 5;
    std::cout << "speed: " << speed << std::endl;

    hts221.setMotorMode(speed);
}
