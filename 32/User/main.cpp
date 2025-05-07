#include "stm32f10x.h"                  // Device header
#include "sys.h"
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "HTS221.h"
#include "delay.h"

int main(void)
{
	/*模块初始化*/
	Stm32_Clock_Init(9);//系统时钟设置
	delay_init(72); 	//延时初始化
	
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

/**
  * 函    数：USART1中断函数
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数为中断函数，无需调用，中断触发后自动执行
  *           函数名为预留的指定名称，可以从启动文件复制
  *           请确保函数名正确，不能有任何差异，否则中断函数将不能进入
  */
void USART1_IRQHandler(void)
{

	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)		//判断是否是USART1的接收事件触发的中断
	{
		uint8_t RxData = USART_ReceiveData(USART1);				//读取数据寄存器，存放在接收的数据变量
		
		
		
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);		//清除标志位
	}
}

/**
  * 函    数：USART2中断函数
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数为中断函数，无需调用，中断触发后自动执行
  *           函数名为预留的指定名称，可以从启动文件复制
  *           请确保函数名正确，不能有任何差异，否则中断函数将不能进入
  */
void USART2_IRQHandler(void)
{

	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)		//判断是否是USART1的接收事件触发的中断
	{
		uint8_t RxData = USART_ReceiveData(USART2);				//读取数据寄存器，存放在接收的数据变量
		
		
		
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);		//清除标志位
	}
}
