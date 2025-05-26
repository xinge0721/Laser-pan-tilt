#include "./Serial.h"
#include <cstring>  // 为std::memcpy添加头文件

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
