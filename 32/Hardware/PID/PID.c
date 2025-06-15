#include "PID.h"

struct PID_TypeDef
{
	float	KP;
	float	KI;
	float	KD;
	float	Bias;
	float	Pwm;
	float	Integral_bias;
	float	Last_Bias;
};

struct PID_TypeDef PID_x = {20,0,10,0,0,0};
struct PID_TypeDef PID_y = {20,0,10,0,0,0};

/*************************************************************************
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM增量
**************************************************************************/
float Position_PID_1(float Position,float Target)
{ 	                                          //增量输出
	 PID_x.Bias=Target-Position;                                  //计算偏差
	 PID_x.Integral_bias+=PID_x.Bias;	                                 //求出偏差的积分
	 PID_x.Pwm=PID_x.KP*PID_x.Bias/100+PID_x.KI*PID_x.Integral_bias/100+PID_x.KD*(PID_x.Bias-PID_x.Last_Bias)/100;       //位置式PID控制器
	 PID_x.Last_Bias=PID_x.Bias;                                       //保存上一次偏差 
	 return PID_x.Pwm;  
}
/*************************************************************************
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM增量
**************************************************************************/


float Position_PID_2(float Position,float Target)
{ 	                                         //增量输出
	 PID_y.Bias=Target-Position;                                  //计算偏差
	 PID_y.Integral_bias+=PID_y.Bias;	                                 //求出偏差的积分
	 PID_y.Pwm=PID_y.KP*PID_y.Bias/100+PID_y.KI*PID_y.Integral_bias/100+PID_y.KD*(PID_y.Bias-PID_y.Last_Bias)/100;       //位置式PID控制器
	 PID_y.Last_Bias=PID_y.Bias;                                       //保存上一次偏差 
	 return PID_y.Pwm;  
}
