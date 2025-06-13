#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "OLED.h"
#include "Serial.h"
#include "Servo.h"//保持这样不要修改
#include "stm32f10x_tim.h"  // 添加TIM库头文件
#include "sys.h"

void HardWare_Init(void)
{
//	MY_NVIC_PriorityGroupConfig(2); //=====中断分组
//	Stm32_Clock_Init(9);    // 系统时钟初始化，参数9表示9倍频，配置为72MHz
	delay_init();		    //延时初始化，72MHz系统时钟
//	Serial_Init(115200);		//串口初始化
//	Servo_PWM_Init();			//数字舵机初始化
	OLED_Init();
//	BEEP_Init();                    //=====蜂鸣器初始化
}


/**
  * 函数：main
  * 描述：主函数入口
  */
int main(void)
{
		delay_init();		    //延时初始化，72MHz系统时钟
		OLED_Init();
		// 保持系统运行
		OLED_Clear();  //清除屏幕
		OLED_Refresh_Gram(); //刷新屏幕
		OLED_ShowString(00,00,"-");      //===显示屏内容显示
		OLED_Refresh_Gram();	
	while (1)
	{
		// 空循环，X轴PWM保持满值
	}
}
