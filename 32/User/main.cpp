#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "HTS221.h"
uint8_t KeyNum;			//定义用于接收按键键码的变量

int main(void)
{
	/*模块初始化*/
	OLED_Init();		//OLED初始化
	Serial_Init();		//串口初始化
	// 舵机初始化
	// 舵机1
	HTS221 duo1(0x00);
	// 舵机2
	HTS221 duo2(0x01);
	
	while (1)
	{
		duo1.turn(1000, 1000);
		
		duo2.turn(1000, 1000);


		delay_ms(10);
	}
}
