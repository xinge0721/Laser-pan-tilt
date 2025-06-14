#ifndef __SERIAL3_H
#define __SERIAL3_H

#include <stdio.h>

extern float x_angle;
extern float y_angle;

void Serial3_Init(int BaudRate);
void Serial3_SendByte(uint8_t Byte);

#endif 
