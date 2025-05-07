#ifndef __HTS221_H
#define __HTS221_H
#include "stm32f10x.h"                  // Device header
#include "Serial.h"

class HTS221
{
public:
	HTS221(uint8_t ID = 0x00,uint8_t size = 10)
	{
		// 初始化ID号
		this->ID = ID;
		this->size = size;
		// 正确初始化数组
		this->date[0] = 0x55;
		this->date[1] = 0x55;
		for(int i = 2; i < size; i++) {
			this->date[i] = 0x00;
		}
	}

    void turn(uint16_t angle, uint16_t speed);
    void stop(void);
    void getAngle(void);
    void setAngleLimit(uint16_t minAngle, uint16_t maxAngle);
	

private:
	uint8_t ID;
	uint8_t date[10];
	uint8_t size;
};

#endif
