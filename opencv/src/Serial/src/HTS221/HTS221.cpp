#include "HTS221.h"

	// 注意事项
	// 注意：在发送舵机控制指令时，需要等待前一条指令运行结束后，再发送
	// 下一条指令，否则旧指令的运行会被打断

	// 转动舵机在指定的速度中，转向指定的角度
	// angle可以是角度值(0-240度)或数据值(0-1000)
	// 如果是角度值，会先转换为数据值
	void HTS221::turn(uint16_t angle, uint16_t speed)
	{
		uint16_t data_value = angle;
		
		// 确保数据在有效范围内
		if (data_value > 1000 || speed > 30000)
		{
			return;
		}

		date[2] = ID;
		// Length
		date[3] = 0x07;
		// Command
		date[4] = 0x01;
		// Data
		// 参数 1：角度的低八位。
		date[5] = data_value & 0xFF;

		// 参数 2：角度的高八位。范围 0~1000，对应舵机角度的 0~240°，即舵机可变化
		// 的最小角度为 0.24度。
		date[6] = data_value >> 8;

		// 参数 3：时间低八位。
		date[7] = speed & 0xFF;

		// 参数 4：时间高八位，时间的范围 0~30000毫秒。该命令发送给舵机，舵机将
		// 在参数时间内从当前角度匀速转动到参数角度。该指令到达舵机后，舵机会立
		// 即转动。
		date[8] = speed >> 8;

		// CRC
		date[9] = ~ (ID + date[3] + date[4] + date[5] + date[6] + date[7] + date[8]);

		// 注释掉发送输出
		// std::cout << "发送: ";
		// for (int i = 0; i < 10; i++) {
		// 	std::cout << (int)date[i] << " ";
		// }
		// std::cout << std::endl;

		globalSerial.sendArray(date, size);
	}

	// 停止舵机
	void HTS221::stop(void)
	{
		date[2] = ID;

		// Length
		date[3] = 0x03;
		// Command
		date[4] = 0x0C;

		date[5] = 0x00;
		date[6] = 0x00;
		date[7] = 0x00;
		date[8] = 0x00;
		// CRC
		// Checksum = ~ (ID + Length + Cmd+ Prm1 + ... Prm N)若括号内的计算和超出 255
		// 则取后 8 位，即对 255 取反。
		date[9] = ~ (ID + date[3] + date[4] + date[5] + date[6] + date[7] + date[8]);

		globalSerial.sendArray(date, 10);
	}

	// 获取舵机角度请求
	// 指令名 SERVO_POS_READ指令值 28数据长度 3
	void HTS221::getAngle(void)
	{
		date[2] = ID;

		// Length
		date[3] = 0x03;
		// Command
		date[4] = 0x0C;

		// CRC
		// Checksum = ~ (ID + Length + Cmd+ Prm1 + ... Prm N)若括号内的计算和超出 255
		// 则取后 8 位，即对 255 取反。
		date[5] = ~ (ID + date[3] + date[4]);

		globalSerial.sendArray(date, 6);	//发送请求
	}

	// 设置舵机角度限制
	// 指令名 SERVO_ANGLE_LIMIT_WRITE指令值 20数据长度 7
	// 参数可以是角度值(0-240度)或数据值(0-1000)
	void HTS221::setAngleLimit(uint16_t minAngle, uint16_t maxAngle)
	{
		uint16_t minData = minAngle;
		uint16_t maxData = maxAngle;
		
		// 确保最小角度小于最大角度
		if (minData >= maxData)
		{
			return;
		}
		
		// 确保角度在有效范围内
		if (minData > 1000 || maxData > 1000)
		{
			return;
		}
		
		date[2] = ID;
		
		// Length
		date[3] = 0x07;
		// Command
		date[4] = 0x14;
		
		// 最小角度低八位
		date[5] = minData & 0xFF;
		// 最小角度高八位
		date[6] = (minData >> 8) & 0xFF;
		// 最大角度低八位
		date[7] = maxData & 0xFF;
		// 最大角度高八位
		date[8] = (maxData >> 8) & 0xFF;
		
		// CRC
		// Checksum = ~ (ID + Length + Cmd+ Prm1 + ... Prm N)若括号内的计算和超出 255
		// 则取后 8 位，即对 255 取反。
		date[9] = ~ (ID + date[3] + date[4] + date[5] + date[6] + date[7] + date[8]);
		
		globalSerial.sendArray(date, 10);
	}

    // 读取舵机角度
	// 指令名 SERVO_POS_READ指令值 28数据长度 3
	// 	**发送格式：**
	// [0x55][0x55][ID][长度=3][0x1C][校验和]
    void HTS221::readAngle(void)
    {
        date[2] = ID;
        date[3] = 0x03;
        date[4] = 0x1C;
        date[5] = ~ (ID + date[3] + date[4]);
        
        // 注释掉发送输出
        // std::cout << "发送: ";
        // for (int i = 0; i < 6; i++) {
        //     std::cout << (int)date[i] << " ";
        // }
        // std::cout << std::endl;
        
        globalSerial.sendArray(date, 6);
    }

    // 舵机返回数据函数
	// **返回格式：**
	// [0x55][0x55][ID][长度=5][0x1C][位置低8位][位置高8位][校验和]
    void HTS221::returnData(uint8_t data)
    {
        // 打印每个接收的字节
        std::cout << (int)data << " ";
        
		switch(cont)
		{
			case 0:
			    if(data == 0x55) cont++;
			    break;
			case 1:
			    if(data == 0x55) cont++;
			    else cont = 0;
			    break;
			case 2:
			    if(data == ID) cont++;
			    else cont = 0;
			    break;
			case 3:
			    if(data == 0x05) cont++;
			    else cont = 0;
			    break;
			case 4:
			    if(data == 0x1C) cont++;
			    else cont = 0;
			    break;
			case 5:
			    this->angle = data & 0xFF;
			    cont++;
			    break;//低八位
			case 6:
			    this->angle |= (data & 0xFF) << 8;
			    cont++;
			    break;//高八位
			case 7:
			{
			    // 判断校验和
			    uint8_t expected = ~ (ID + 0x05 + 0x1C + (uint8_t)(angle & 0xFF) + (uint8_t)((angle >> 8) & 0xFF));
			    
			    if(data == expected) {
			        // 创建AngleData对象转换角度
			        int angleValue = angleConverter.dataToAngle(this->angle);
			        
			        std::cout << "舵机" << (int)ID << "角度值: " << angleValue << "° (数据: " << this->angle << ")" << std::endl;
			        return_flag = true;
			    } else {
			        std::cout << "舵机" << (int)ID << "校验错误" << std::endl;
			    }
			    cont = 0;
			    break;
			}
			default:
			    cont = 0;
			    break;
		}
    }

	// 滑块控制函数实现
	// 根据滑块的百分比值调整舵机角度和速度
	// percent参数范围为0-100，对应角度0-240度
	void HTS221::sliderControl(int angle_percent, int speed_percent)
	{
		// 限制百分比范围为0-100
		if (angle_percent < 0) angle_percent = 0;
		if (angle_percent > 100) angle_percent = 100;
		if (speed_percent < 0) speed_percent = 0;
		if (speed_percent > 100) speed_percent = 100;
		
		// 将百分比转换为实际角度和速度值
		uint16_t angle = (uint16_t)((angle_percent * 240) / 100);  // 0-100% 映射到 0-240度
		
		// 使用AngleData类转换角度到数据值
		uint16_t data = angleConverter.angleToData(angle);
		
		uint16_t speed = (uint16_t)((speed_percent * 30000) / 100); // 0-100% 映射到 0-30000
		
		// 至少保证有最小速度
		if (speed < 50) speed = 50;
		
		// 调用turn函数执行舵机控制
		turn(data, speed);
	}



// 指令名 SERVO_OR_MOTOR_MODE_WRITE指令值 29数据长度 7：
// 参数 1：舵机模式，范围0或 1，0代表位置控制模式，1代表电机控制模式，
// 默认值 0，
// 参数 2：空值，
// 参数 3：转动速度值的低八位。
// 参数 4：转动速度值的高八位。范围-1000~1000，只在电机控制模式时有效，控
// 制电机的转速，该值为负值代表反转，正值代表正转。写入的模式和速度
// 掉电保存。
// 注意：
// 1由于转动速度为 signedshort int型数据，所以发送改指令包之前先将
// 该 数据强制转换成 unsignedshort int型数据在进行数据传输。
// 2在指令中，参数3与参数4以16进制补码形式表示。计算方式：正数为
// 16进制原码，负数为取二进制数按位取反再加1。例如：1000表示为：
// 03E8,500表示为：01F4，-1000表示为：FC18。具体计算方法可搜索补码相关
// 知识进行了解。
// 举例：
// 1）设置总线舵机1为电机控制模式，转动速度为100，如下：
// 55 55 01 07 1D 01 00 64 00 crc

// 直接开启电机模式
void HTS221::setMotorMode(int16_t speed)
{
	date[2] = ID;
	date[3] = 0x07;
	date[4] = 0x1D;
	date[5] = 0x01;
	date[6] = 0x00;
	
	// 处理速度值，确保正确处理负数补码
	uint16_t speed_value = static_cast<uint16_t>(speed);
	date[7] = speed_value & 0xFF;
	date[8] = speed_value >> 8;
	date[9] = ~ (ID + 0x07 + 0x1D + 0x01 + 0x00 + (speed_value & 0xFF) + (speed_value >> 8));

	globalSerial.sendArray(date, 10);
}

// AngleData结构体函数实现
// 数据转换
// 根据数据手册所示范围 0~1000，对应舵机角度的 0~240°
// 所以需要将数据转换为角度
int AngleData::dataToAngle(int data)
{
	// 使用浮点数计算以提高精度
	return (uint16_t)((float)data * 0.24f);  // 1000对应240度
}

// 将角度转换为数据
int AngleData::angleToData(int angle)
{
	// 使用浮点数计算以提高精度，0-240度映射到0-1000
	return (uint16_t)((float)angle * (1000.0f / 240.0f));
}


