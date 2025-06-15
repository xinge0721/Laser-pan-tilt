#include "speed.h"


// 先尝试 开环电机控制
// 不写类

// 用于跟踪不同舵机速度持续时间的静态变量
struct ServoState {
    int16_t last_speed = 0;
    int duration_counter = 0;
};

// 创建两个舵机的状态跟踪器 (索引0为X轴，索引1为Y轴)
static ServoState servo_states[2];
static const int THRESHOLD = 10; // 持续10次后开始提升速度

// 开环电机模式控制
// 参数一：距离（像素点）
// 参数二：舵机ID（1=X轴，2=Y轴）
// 速度范围：-1000-1000
// 特别注意当speed在-150~150之间时，根本不会转了
// 而零数据舵机不会响应，所以得给一个很小的数据，让舵机"停下"
// 特别注意，停止指令hts221.stop()，只有在位置控制模式有效，电机模式不论怎么发，舵机都不管你
int16_t setSpeed(int juli, int servo_id)
{
    // 确保servo_id是有效的 (1或2)，并转换为数组索引 (0或1)
    int index = (servo_id == 1) ? 0 : 1;
    
    // 定义距离范围和对应的速度值
    const struct {
        int min_dist;
        int max_dist;
        int16_t speed;
    } speed_map[] = {
        {-1000, -150, -210},
        {-150, -50, -200},
        {-50, -30, -190},
        {-30, -20, -180},
        {-20, -10, -150},
        {-10, -5, -120},
        {-5, 5, 5},     // 中间区域，几乎静止
        {5, 10, 120},
        {10, 20, 150},
        {20, 30, 180},
        {30, 50, 190},
        {50, 150, 200},
        {150, 1000, 210}
    };
    
    // 查找适合的速度区间
    int16_t speed = 5; // 默认值
    for (const auto& range : speed_map) {
        if (juli > range.min_dist && juli <= range.max_dist) {
            speed = range.speed;
            break;
        }
    }
    
    // 获取当前舵机的状态引用
    ServoState& state = servo_states[index];
    
    // 处理120的特殊情况 - 如果连续多次都是120，逐渐增加到150
    if (speed == 120) {
        if (state.last_speed == 120) {
            state.duration_counter++;
            if (state.duration_counter > THRESHOLD) {
                // 计算提升后的速度，最大提升到150
                int boost = (state.duration_counter - THRESHOLD) * 3; // 每次提升3
                if (boost > 30) boost = 30; // 最多提升到150 (120+30)
                speed = 120 + boost;
            }
        } else {
            state.duration_counter = 0; // 重置计数器
        }
    } else if (speed == -120) {
        // 同样处理-120的情况
        if (state.last_speed == -120) {
            state.duration_counter++;
            if (state.duration_counter > THRESHOLD) {
                int boost = (state.duration_counter - THRESHOLD) * 3;
                if (boost > 30) boost = 30;
                speed = -120 - boost; // 负方向提升到-150
            }
        } else {
            state.duration_counter = 0;
        }
    } else {
        state.duration_counter = 0; // 其他速度重置计数器
    }
    
    state.last_speed = speed; // 保存这次的速度值
    
    std::cout << "Servo " << servo_id << " speed: " << speed << std::endl;
    return speed;
}
