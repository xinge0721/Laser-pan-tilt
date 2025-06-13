#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f10x.h"
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
void Servo_PWM_Init(void);

/**
  * 函数：Servo::SetAngle
  * 描述：设置舵机角度，支持0.1度精度
  * 参数：angle - 角度值（0-180度，支持小数精度0.1度）
  * 返回：无
  * 备注：标准舵机通常是0.5ms-2.5ms对应0-180度
  *       对应PWM值为1000-5000 (在2MHz计数频率下)
  */
void SetAngle(float angle,uint8_t _channel);

#endif 
