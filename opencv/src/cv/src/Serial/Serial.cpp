#include "./Serial.h"
#include <cstring>  // 为std::memcpy添加头文件
#include <cmath>   // 为std::abs添加头文件

myserial::myserial() : isConnected(false), portName(""), baudRate(0)
{
}

myserial::~myserial()
{
    close();
}

bool myserial::init(const std::string& port, int baudrate)
{
    // 关闭之前可能打开的串口
    if (isConnected)
    {
        close();
    }

    portName = port;
    baudRate = baudrate;

    try
    {
        // 配置串口
        ser.setPort(portName);
        ser.setBaudrate(baudRate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(timeout);
        
        // 设置串口参数
        ser.setBytesize(serial::eightbits);    // 数据位 = 8
        ser.setParity(serial::parity_none);    // 无校验位
        ser.setStopbits(serial::stopbits_one); // 停止位 = 1
        ser.setFlowcontrol(serial::flowcontrol_none); // 无流控制
        
        // 打开串口
        ser.open();
        
        // 检查串口是否成功打开
        if (ser.isOpen())
        {
            isConnected = true;
            std::cout << "串口 " << portName << " 已成功打开，波特率：" << baudRate << std::endl;
            return true;
        }
        else
        {
            std::cout << "串口无法打开" << std::endl;
            return false;
        }
    }
    catch (serial::IOException& e)
    {
        std::cout << "串口打开异常: " << e.what() << std::endl;
        return false;
    }
    catch (std::exception& e)
    {
        std::cout << "发生异常: " << e.what() << std::endl;
        return false;
    }
}

bool myserial::send(const std::string& data)
{
    if (!isConnected || !ser.isOpen())
    {
        std::cout << "串口未连接" << std::endl;
        return false;
    }

    try
    {
        // 发送数据
        size_t bytesWritten = ser.write(data);
        
        if (bytesWritten != data.size())
        {
            std::cout << "发送数据不完整，已发送 " << bytesWritten << " 字节，预期发送 " << data.size() << " 字节" << std::endl;
            return false;
        }
        
        return true;
    }
    catch (std::exception& e)
    {
        std::cout << "发送数据异常: " << e.what() << std::endl;
        return false;
    }
}

bool myserial::sendArray(const uint8_t* data, size_t length)
{
    if (!isConnected || !ser.isOpen())
    {
        std::cout << "串口未连接" << std::endl;
        return false;
    }

    try
    {
        // 直接发送字节数组
        size_t bytesWritten = ser.write(data, length);
        
        if (bytesWritten != length)
        {
            std::cout << "发送数组不完整，已发送 " << bytesWritten << " 字节，预期发送 " << length << " 字节" << std::endl;
            return false;
        }
        
        return true;
    }
    catch (std::exception& e)
    {
        std::cout << "发送数组异常: " << e.what() << std::endl;
        return false;
    }
}

std::string myserial::receive(size_t size)
{
    if (!isConnected || !ser.isOpen())
    {
        std::cout << "串口未连接" << std::endl;
        return "";
    }

    try
    {
        // 检查是否有数据可读
        if (ser.available() == 0)
        {
            return "";
        }
        
        // 读取数据
        std::string buffer = ser.read(size);
        return buffer;
    }
    catch (std::exception& e)
    {
        std::cout << "接收数据异常: " << e.what() << std::endl;
        return "";
    }
}

std::vector<uint8_t> myserial::receiveArray(size_t size)
{
    std::vector<uint8_t> buffer;
    
    if (!isConnected || !ser.isOpen())
    {
        std::cout << "串口未连接" << std::endl;
        return buffer;  // 返回空vector
    }

    try
    {
        // 检查是否有数据可读
        size_t available = ser.available();
        if (available == 0)
        {
            return buffer;  // 返回空vector
        }
        
        // 确定要读取的大小，不超过available或请求的size
        size_t readSize = std::min(available, size);
        
        // 预分配buffer大小
        buffer.resize(readSize);
        
        // 读取数据
        size_t bytesRead = ser.read(buffer.data(), readSize);
        
        // 调整为实际读取的大小
        if (bytesRead < readSize)
        {
            buffer.resize(bytesRead);
        }
    }
    catch (std::exception& e)
    {
        std::cout << "接收数组异常: " << e.what() << std::endl;
    }
    
    return buffer;
}

// 实现readBuffer方法
int myserial::readBuffer(uint8_t* buffer, size_t size)
{
    if (!isConnected || !ser.isOpen())
    {
        return 0;  // 串口未连接，返回读取0字节
    }

    try
    {
        // 检查是否有数据可读
        size_t available = ser.available();
        if (available == 0)
        {
            return 0;  // 没有可读数据，返回读取0字节
        }
        
        // 确定要读取的大小，不超过available或请求的size
        size_t readSize = std::min(available, size);
        
        // 直接读取到提供的缓冲区
        size_t bytesRead = ser.read(buffer, readSize);
        
        return static_cast<int>(bytesRead);
    }
    catch (std::exception& e)
    {
        std::cout << "读取缓冲区异常: " << e.what() << std::endl;
        return -1;  // 读取异常，返回-1
    }
}

void myserial::close()
{
    if (isConnected && ser.isOpen())
    {
        ser.close();
        isConnected = false;
        std::cout << "串口已关闭" << std::endl;
    }
}

bool myserial::isOpen() const
{
    return isConnected && ser.isOpen();
}

/**
 * 发送Point点数据到32位单片机
 * 数据包格式:
 * - 包头(1字节): 0xAA
 * - 数据位一(1字节): x轴高八位（最高位表示符号，1为负数）
 * - 数据位二(1字节): x轴低八位
 * - 数据位三(1字节): y轴高八位（最高位表示符号，1为负数）
 * - 数据位四(1字节): y轴低八位
 * - 校验位(1字节): 四个数据位相加取余256
 * - 包尾(1字节): 0x55
 * 
 * @param point 要发送的点坐标
 * @return 发送是否成功
 */
bool myserial::sendPoint(int x, int y) {
    // 提取Point中的x和y坐标
    
    // 创建7字节的数据包缓冲区
    uint8_t buffer[7];
    
    // 设置包头: 0xAA
    buffer[0] = 0xAA;
    
    // 处理X坐标
    bool x_negative = x < 0;
    uint16_t abs_x = std::abs(x);
    uint8_t x_high = (abs_x >> 8) & 0x7F;  // 取高8位，并清除最高位
    uint8_t x_low = abs_x & 0xFF;          // 取低8位
    
    // 处理Y坐标
    bool y_negative = y < 0;
    uint16_t abs_y = std::abs(y);
    uint8_t y_high = (abs_y >> 8) & 0x7F;  // 取高8位，并清除最高位
    uint8_t y_low = abs_y & 0xFF;          // 取低8位
    
    // 如果原值为负，在高8位的最高位设置为1
    if (x_negative) x_high |= 0x80;
    if (y_negative) y_high |= 0x80;
    
    // 填充缓冲区
    buffer[1] = x_high;
    buffer[2] = x_low;
    buffer[3] = y_high;
    buffer[4] = y_low;
    
    // 计算校验和: 四个数据位相加取余256
    buffer[5] = (x_high + x_low + y_high + y_low) & 0xFF;
    
    // 设置包尾: 0x55
    buffer[6] = 0x55;
    
    // 发送数据包
    return sendArray(buffer, sizeof(buffer));
}

/*
 * 示例1: 发送点 (100, 200)
 * x = 100 (0x0064), y = 200 (0x00C8)
 * x_high = 0x00, x_low = 0x64
 * y_high = 0x00, y_low = 0xC8
 * 校验和 = (0x00 + 0x64 + 0x00 + 0xC8) & 0xFF = 0x2C
 * 数据包: [0xAA, 0x00, 0x64, 0x00, 0xC8, 0x2C, 0x55]
 *
 * 示例2: 发送点 (-100, 200)
 * x = -100, abs(x) = 100 (0x0064), y = 200 (0x00C8)
 * x_high = 0x00 | 0x80 = 0x80, x_low = 0x64
 * y_high = 0x00, y_low = 0xC8
 * 校验和 = (0x80 + 0x64 + 0x00 + 0xC8) & 0xFF = 0xAC
 * 数据包: [0xAA, 0x80, 0x64, 0x00, 0xC8, 0xAC, 0x55]
 *
 * 示例3: 发送点 (1000, -2000)
 * x = 1000 (0x03E8), abs(y) = 2000 (0x07D0)
 * x_high = 0x03, x_low = 0xE8
 * y_high = 0x07 | 0x80 = 0x87, y_low = 0xD0
 * 校验和 = (0x03 + 0xE8 + 0x87 + 0xD0) & 0xFF = 0x42
 * 数据包: [0xAA, 0x03, 0xE8, 0x87, 0xD0, 0x42, 0x55]
 */
