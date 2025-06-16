#include "stm32f10x.h"                  // Device header
#include "delay.h"
#include "OLED.h"
#include "Serial.h"
#include "Servo.h"//保持这样不要修改
#include "stm32f10x_tim.h"  // 添加TIM库头文件
#include "sys.h"
#include "Serial3.h"
#include "PID.h"
#include "APP.h"
float Velocity1 = 0,Velocity2 = 0;     //电机PWM变量
float Position1 = 3000,Position2 = 3000; //舵机当前位置
unsigned char Send_Count; //串口需要发送的数据个数
/**
  * 函数：main
  * 描述：主函数入口
  */
int main(void)
{
	MY_NVIC_PriorityGroupConfig(2); //=====中断分组
	Stm32_Clock_Init(9);    // 系统时钟初始化，参数9表示9倍频，配置为72MHz
	delay_init();		    		//延时初始化，72MHz系统时钟
	Serial_Init(115200);		//串口初始化
	Serial3_Init(115200);		//串口初始化
	Servo_PWM_Init();			  //数字舵机初始化


	TIM2_Init();
	OLED_Init();

	while (1)
	{

		delay_ms(10);
		OLED_ShowString(0,0,"X_pulse:");
		OLED_ShowString(0,1,"Y_pulse:");
		OLED_ShowNumber(9,0,x_pulse,5);
		OLED_ShowNumber(9,1,y_pulse,5);
//		delay_ms(500);
//		SetPulse(x_pulse,1);
//		x_pulse+=100;
//		delay_ms(500);
		
		OLED_Refresh_Gram();	//非常重要，若是不使用，单纯使用OLED_ShowString()，则不会显示内容
//		delay_ms(10);
//		DataScope_Get_Channel_Data(x_pulse*0.01, 1 );//目标数据
//		DataScope_Get_Channel_Data(Position1*0.01, 2 );//当前左轮数据
//		DataScope_Get_Channel_Data(Position2*0.01, 3 );//当前右轮数据
//		DataScope_Get_Channel_Data(Velocity1, 4 );//当前右轮数据
//		DataScope_Get_Channel_Data(Velocity2, 5);//当前右轮数据

//			
//		Send_Count = DataScope_Data_Generate(10);
//		for(int j = 0 ; j < Send_Count; j++) 
//		{
//			Serial_SendByte( DataScope_OutPut_Buffer[j]); //发送到上位机
//		}
//		 delay_ms(50);
		}
}

float xianzhi_pulse(float x)
{
	if(x<2000)
		return 2000;
	else if(x>4000)
		return 4000;
	else
		return x;
}


void TIM2_IRQHandler(void)
{
    // 检查是否发生了更新中断
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
				x_pulse=xianzhi_pulse(x_pulse);
				y_pulse=xianzhi_pulse(y_pulse);
				
				Velocity1=Position_PID_1(Position1,x_pulse);
				Velocity2=Position_PID_2(Position2,y_pulse);
				Position1+=Velocity1;
				Position2+=Velocity2;

				SetPulse((uint16_t)Position1,1);
				 SetPulse((uint16_t)Position2,2);
				TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
} 


