#ifndef __SERIAL3_H
#define __SERIAL3_H

#include <stdio.h>

#include "stm32f10x.h"                  // Device header

void Serial3_Init(int BaudRate);
void Serial3_SendByte(uint8_t Byte);

extern float x_angle;
extern float y_angle;

void RX_Data_Process(uint8_t RxData);

#endif 
