#ifndef __HTS221_H
#define __HTS221_H
#include "./../Serial/Serial.h"
#include <iostream>
#define uint8_t unsigned char
#define int16_t short int
#define int8_t char
#define uint32_t unsigned long int
#define int32_t long int

class HTS221
{
public:
    // 构造函数
    // ID: 舵机ID号
    // size: 数据包大小
    // 注意：size必须大于6，否则无法正确初始化数据包
    // 通信协议最短是6个字节，所以size至少为6
    // 通信协议长度一般为10个字节，所以size的缺省值为10
	HTS221(uint8_t ID = 0x00,uint8_t size = 10,const std::string& port="/dev/ttyUSB0", int baudrate=115200)
    :ID(ID)
    ,size(size)
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
        if (!serial.init(port, baudrate)) {
            std::cerr << "串口初始化失败！" << std::endl;
        } else {
            std::cout << "串口初始化成功！" << std::endl;
        }

	}

    // 析构函数
    ~HTS221()
    {
        delete[] date;
        // 关闭串口
        serial.close();
    }

    void turn(uint16_t angle, uint16_t speed);
    void stop(void);
    void getAngle(void);
    void setAngleLimit(uint16_t minAngle, uint16_t maxAngle);
	
    // 滑块控制函数
    // 根据滑块值调整舵机角度和速度
    // angle_percent: 角度百分比，范围0-100，对应舵机角度0-1000
    // speed_percent: 速度百分比，范围0-100，对应舵机速度0-30000
    void sliderControl(int angle_percent, int speed_percent);

private:
	const uint8_t ID;
	uint8_t* date;
	const uint8_t size;
    myserial serial;
};

// 关于角度数据处理结构体
struct AngleData
{
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;

    // 数据转换
    // 根据数据手册所示范围 0~1000，对应舵机角度的 0~240°
    // 所以需要将数据转换为角度
    // 注意：这种转换是线性映射，每个单位数据对应0.24度角度
    void dataToAngle(void);

    // 将角度转换为数据
    // 注意：这种转换是线性映射，每个度角度对应约4.17个单位数据
    void angleToData(void);

    // 处理摄像头数据，转化为舵机可用参数
    // 参数一：目标的中心点x坐标
    // 参数二：目标的中心点y坐标
    // 摄像头数据为像素点的相对坐标，而非舵机的坐标
    void processData(uint16_t centerX, uint16_t centerY);
};




#endif
