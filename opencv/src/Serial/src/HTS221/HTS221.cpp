#include "HTS221.h"

	// 注意事项
	// 注意：在发送舵机控制指令时，需要等待前一条指令运行结束后，再发送
	// 下一条指令，否则旧指令的运行会被打断


	// 转动舵机在指定的速度中，转向指定的角度
	void HTS221::turn(uint16_t angle, uint16_t speed)
	{
		// 确保数据在有效范围内
		if (angle > 1000 || speed > 30000)
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
		date[5] = angle & 0xFF;

		// 参数 2：角度的高八位。范围 0~1000，对应舵机角度的 0~240°，即舵机可变化
		// 的最小角度为 0.24度。
		date[6] = angle >> 8;

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
		date[4] = 0x1C;

		date[5] = 0x00;
		date[6] = 0x00;
		date[7] = 0x00;
		date[8] = 0x00;
		// CRC
		// Checksum = ~ (ID + Length + Cmd+ Prm1 + ... Prm N)若括号内的计算和超出 255
		// 则取后 8 位，即对 255 取反。
		date[9] = ~ (ID + date[3] + date[4] + date[5] + date[6] + date[7] + date[8]);

		globalSerial.sendArray(date, 10);	//发送请求
	}

	// 设置舵机角度限制
	// 指令名 SERVO_ANGLE_LIMIT_WRITE指令值 20数据长度 7
	void HTS221::setAngleLimit(uint16_t minAngle, uint16_t maxAngle)
	{
		// 确保最小角度小于最大角度
		if (minAngle >= maxAngle)
		{
			return;
		}
		
		// 确保角度在有效范围内
		if (minAngle > 1000 || maxAngle > 1000)
		{
			return;
		}
		
		date[2] = ID;
		
		// Length
		date[3] = 0x07;
		// Command
		date[4] = 0x14;
		
		// 最小角度低八位
		date[5] = minAngle & 0xFF;
		// 最小角度高八位
		date[6] = (minAngle >> 8) & 0xFF;
		// 最大角度低八位
		date[7] = maxAngle & 0xFF;
		// 最大角度高八位
		date[8] = (maxAngle >> 8) & 0xFF;
		
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
			    extern int target_angle; // 引用全局目标角度变量
			    
			    // 判断校验和
			    uint8_t expected = ~ (ID + 0x05 + 0x1C + (uint8_t)(angle & 0xFF) + (uint8_t)((angle >> 8) & 0xFF));
			    
			    if(data == expected) {
			        // 打印当前角度值和目标角度
			        std::cout << "角度值: " << this->angle << " 目标: " << target_angle << std::endl;
			        return_flag = true;
			    } else {
			        std::cout << " 校验错误" << std::endl;
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
	void HTS221::sliderControl(int angle_percent, int speed_percent)
	{
		// 限制百分比范围为0-100
		if (angle_percent < 0) angle_percent = 0;
		if (angle_percent > 100) angle_percent = 100;
		if (speed_percent < 0) speed_percent = 0;
		if (speed_percent > 100) speed_percent = 100;
		
		// 将百分比转换为实际角度和速度值
		uint16_t angle = (uint16_t)((angle_percent * 1000) / 100);  // 0-100% 映射到 0-1000
		uint16_t speed = (uint16_t)((speed_percent * 30000) / 100); // 0-100% 映射到 0-30000
		
		// 至少保证有最小速度
		if (speed < 50) speed = 50;
		
		// 调用turn函数执行舵机控制
		turn(angle, speed);
	}


// AngleData结构体函数实现
// 数据转换
// 根据数据手册所示范围 0~1000，对应舵机角度的 0~240°
// 所以需要将数据转换为角度
// 注意：这种转换是线性映射，每个单位数据对应0.24度角度
void AngleData::dataToAngle(void)
{
	// 使用浮点数计算以提高精度
	x = (uint16_t)((float)x * 0.24f);
	y = (uint16_t)((float)y * 0.24f);
}

// 将角度转换为数据
// 注意：这种转换是线性映射，每个度角度对应约4.17个单位数据
void AngleData::angleToData(void)
{
	// 使用浮点数计算以提高精度
	x = (uint16_t)((float)x * 4.17f);
	y = (uint16_t)((float)y * 4.17f);
	
	// 确保数据不超过最大值1000
	if (x > 1000) x = 1000;
	if (y > 1000) y = 1000;
}

// 处理摄像头数据，转化为舵机可用参数
// 参数一：目标的中心点x坐标
// 参数二：目标的中心点y坐标
// 摄像头数据为像素点的相对坐标，而非舵机的坐标
void AngleData::processData(uint16_t centerX, uint16_t centerY)
{
	// 根据设定的范围限制坐标
	// X轴范围：613~823
	// Y轴范围：562~750
	if(centerX < 613)
	{
		centerX = 613;
	}
	else if(centerX > 823)
	{
		centerX = 823;
	}
	
	if(centerY < 562)
	{
		centerY = 562;
	}
	else if(centerY > 750)
	{
		centerY = 750;
	}
	
	x = centerX;
	y = centerY;
}

