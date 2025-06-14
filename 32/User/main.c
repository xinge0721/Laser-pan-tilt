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
float Velocity1,Velocity2;     //电机PWM变量
float Position1=0,Position2=0; //舵机当前位置
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
	TIM_SetCompare3(TIM3, 0);
	TIM_SetCompare4(TIM3, 0);
//	delay_ms(500);
//	delay_ms(500);
//	delay_ms(500);
//	delay_ms(500);
  TIM2_Init();
	OLED_Init();

	while (1)
	{

		delay_ms(10);
		OLED_ShowString(0,0,"X:");
		OLED_ShowString(0,1,"Y:");
		OLED_ShowNumber(2,0,x_angle,4);
		OLED_ShowNumber(2,1,y_angle,4);
		OLED_ShowString(0,3,"Position1:");
		OLED_ShowString(0,4,"Position2:");
		OLED_ShowNumber(10,3,Position1,4);
		OLED_ShowNumber(10,4,Position2,4);
		
		OLED_Refresh_Gram();	//非常重要，若是不使用，单纯使用OLED_ShowString()，则不会显示内容
		delay_ms(10);
//				DataScope_Get_Channel_Data(750, 1 );//目标数据
//	DataScope_Get_Channel_Data(Position1, 2 );//当前左轮数据
//	DataScope_Get_Channel_Data(Position2, 3 );//当前右轮数据
//	DataScope_Get_Channel_Data(Velocity1, 4 );//当前右轮数据
//	DataScope_Get_Channel_Data(Velocity2, 5);//当前右轮数据
//		DataScope_Get_Channel_Data(1560, 6 );//当前右轮数据
//		DataScope_Get_Channel_Data(ADD, 7 );//当前差速数据
//		DataScope_Get_Channel_Data(zangle, 8 );//当前陀螺仪z轴数据
			
//		Send_Count = DataScope_Data_Generate(10);
//		for(int j = 0 ; j < Send_Count; j++) 
//		{
//			Serial3_SendByte( DataScope_OutPut_Buffer[j]); //发送到上位机
//		}
//		 delay_ms(50);
		}
}
int cont;
void TIM2_IRQHandler(void)
{
    // 检查是否发生了更新中断
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
				
				Velocity1=Position_PID_1(Position1,x_angle);
				Velocity2=Position_PID_2(Position2,y_angle);
				Position1+=Velocity1;
				Position2+=Velocity2;
				TIM_SetCompare3(TIM3, Position1);
				TIM_SetCompare4(TIM3, Position2);

				TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
} 


