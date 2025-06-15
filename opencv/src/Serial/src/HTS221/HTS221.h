#ifndef __HTS221_H
#define __HTS221_H
#include "./../Serial/Serial.h"
#include <iostream>
#define uint8_t unsigned char
#define int16_t short int
#define int8_t char
#define uint32_t unsigned long int
#define int32_t long int

// 关于角度数据处理结构体
typedef struct AngleData
{
    // 数据转换
    // 根据数据手册所示范围 0~1000，对应舵机角度的 0~240°
    // 所以需要将数据转换为角度
    // 注意：这种转换是线性映射，每个单位数据对应0.24度角度
    int dataToAngle(int data);

    // 将角度转换为数据
    // 注意：这种转换是线性映射，每个度角度对应约4.17个单位数据
    int angleToData(int angle);

    // 处理摄像头数据，转化为舵机可用参数
    // 参数一：目标的中心点x坐标
    // 参数二：目标的中心点y坐标
    // 摄像头数据为像素点的相对坐标，而非舵机的坐标
    void processData(uint16_t centerX, uint16_t centerY);
}AngleData;

// 声明全局串口对象（在main.cpp中定义）
extern myserial globalSerial;

class HTS221
{
public:
    // 构造函数
    // ID: 舵机ID号
    // size: 数据包大小
    // 注意：size必须大于6，否则无法正确初始化数据包
    // 通信协议最短是6个字节，所以size至少为6
    // 通信协议长度一般为10个字节，所以size的缺省值为10
	HTS221(uint8_t ID = 0x00,uint8_t size = 10)
    :ID(ID)
    ,size(size)
    ,kp(1.0)
    ,kd(0.1)
    ,last_Err(0)
    ,pwm(0)
    {
        if(size < 6)
        {
            return;
        }
        date = new uint8_t[size];
		// 正确初始化数组
		date[0] = 0x55;
		date[1] = 0x55;
		for(int i = 2; i < size; i++) {
			date[i] = 0x00;
		}
        
        cont = 0;  // 初始化cont变量
	}

    // 析构函数
    ~HTS221()
    {
        delete[] date;
    }
    
    //转动舵机
    void turn(uint16_t angle, uint16_t speed);  
    //停止舵机
    void stop(void);
    void getAngle(void);
    void setAngleLimit(uint16_t minAngle, uint16_t maxAngle);
	
    // 读取舵机角度
    void readAngle(void);

    // 舵机返回数据函数
    void returnData(uint8_t data);

    // 滑块控制函数
    // 根据滑块值调整舵机角度和速度
    // angle_percent: 角度百分比，范围0-100，对应舵机角度0-1000
    // speed_percent: 速度百分比，范围0-100，对应舵机速度0-30000
    void sliderControl(int angle_percent, int speed_percent);
    
    // 获取当前舵机角度
    uint16_t getCurrentAngle() const { return angle; }
    void setMotorMode(int16_t speed);

    // PID控制相关参数
    float kp;       // 比例系数
    float kd;       // 微分系数
    int last_Err;   // 上一次误差
    int pwm;        // 控制PWM值
    
    bool return_flag; // 返回标志位
public:  // 使angle可直接访问
	const uint8_t ID;
    uint16_t angle; // 当前角度

private:
	uint8_t* date;
	const uint8_t size;
    uint16_t speed;
    uint8_t cont;
    AngleData angleConverter; // 重新添加angleConverter成员
};

#endif
