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
int16_t setSpeed(int juli)
{
    // 定义距离范围和对应的速度值
    const struct {
        int min_dist;
        int max_dist;
        int16_t speed;
    } speed_map[] = {
        {-1000, -150, -150},
        {-150, -50, -150},
        {-50, -30, -150},
        {-30, -20, -150},
        {-20, -10, -150},
        {-10, -5, -150},
        {-5, 5, 5},     // 中间区域，几乎静止
        {5, 10, 150},
        {10, 20, 150},
        {20, 30, 150},
        {30, 50, 150},
        {50, 150, 150},
        {150, 1000, 150}
    };
    
    // 查找适合的速度区间
    int16_t speed = 5; // 默认值
    for (const auto& range : speed_map) {
        if (juli > range.min_dist && juli <= range.max_dist) {
            speed = range.speed;
            break;
        }
    }
    
    std::cout << "speed: " << speed << std::endl;
    return speed;
}
