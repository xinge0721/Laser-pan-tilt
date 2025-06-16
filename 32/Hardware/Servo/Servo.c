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
  * 函数：SetPulse
  * 描述：设置舵机脉宽
  * 参数：pulse - 脉宽值 (1500-4000)
  * 返回：无
  * 备注：标准舵机通常是0.5ms-2.5ms对应0-240度
  *       对应PWM值为1000-5000 (在2MHz计数频率下)
  *       此处限制范围为1500-4000
  */
void SetPulse(uint16_t pulse,uint8_t _channel)
{
    if (pulse < 1500) pulse = 1500;
    if (pulse > 4000) pulse = 4000;
    
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
