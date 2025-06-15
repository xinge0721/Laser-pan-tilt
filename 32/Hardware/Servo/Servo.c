#include "Servo.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"

/**
  * 函数：Servo_PWM_Init
  * 描述：初始化TIM3的通道3和通道4为PWM输出，用于控制两个数字舵机
  * 参数：无
  * 返回：无
  * 备注：TIM3_CH3 - PB0，TIM3_CH4 - PB1
  */
void Servo_PWM_Init(void)
{
    // 1. 开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    // 2. 配置GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; // PB0和PB1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 3. 配置定时器基础参数
    // 舵机需要周期为20ms的PWM信号
    // 对于更高精度控制，使用更高分辨率的计数
    // 72MHz / 36 = 2MHz，计数频率2MHz
    // 2MHz / 40000 = 50Hz (周期20ms)
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 40000 - 1;         // ARR
    TIM_TimeBaseStructure.TIM_Prescaler = 36 - 1;         // PSC
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    // 4. 配置PWM模式
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    
    // 通道3配置 - 初始位置(1.5ms = 3000)
    TIM_OCInitStructure.TIM_Pulse = 3000;  // 1.5ms脉宽
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    // 通道4配置 - 初始位置(1.5ms = 3000)
    TIM_OCInitStructure.TIM_Pulse = 3000;  // 1.5ms脉宽
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    // 5. 使能定时器
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}


/**
  * 函数：Servo::SetAngle
  * 描述：设置舵机角度，支持0.1度精度
  * 参数：angle - 角度值（0-240度，支持小数精度0.1度）
  * 返回：无
  * 备注：标准舵机通常是0.5ms-2.5ms对应0-240度
  *       对应PWM值为1000-5000 (在2MHz计数频率下)
  */
void SetAngle(float angle,uint8_t _channel)
{
    if (angle < 30.0f) angle = 30.0f;
    if (angle > 180.0f) angle = 180.0f;
    
    // 将角度转换为脉宽，每0.1度都可以正确映射
    // 0度 = 0.5ms = 1000 counts
    // 240度 = 2.5ms = 5000 counts
    // 精确计算每0.1度对应的脉宽增量
    float pulseWidth = 1000.0f + (angle * 16.666667f);  // 更精确的映射: (5000-1000)/240 = 16.666...
    uint16_t pulse = (uint16_t)(pulseWidth + 0.5f);  // 加0.5并取整，确保舍入精确
    
    // 根据通道设置PWM值
    if (_channel == 1)
    {
        TIM_SetCompare3(TIM3, pulse);
    }
    else if (_channel == 2)
    {
        TIM_SetCompare4(TIM3, pulse);
    }
} 
