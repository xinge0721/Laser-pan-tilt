#ifndef __serial_H_
#define __serial_H_

#include <serial/serial.h>
#include <string>
#include <iostream>
#include <vector>

class myserial
{
public:
    myserial();
    ~myserial();
    
    // 初始化串口
    bool init(const std::string& port = "/dev/ttyUSB0", int baudrate = 115200);
    
    // 发送字符串数据
    bool send(const std::string& data);
    
    // 发送字节数组
    bool sendArray(const uint8_t* data, size_t length);
    
    // 发送任意类型的数组数据
    template<typename T>
    bool sendData(const T* data, size_t length) {
        if (!isConnected || !ser.isOpen()) {
            std::cout << "串口未连接" << std::endl;
            return false;
        }
        
        try {
            // 计算字节大小
            size_t byteSize = length * sizeof(T);
            
            // 转换为字节数组
            uint8_t* byteArray = new uint8_t[byteSize];
            std::memcpy(byteArray, data, byteSize);
            
            // 发送数据
            size_t bytesWritten = ser.write(byteArray, byteSize);
            
            // 释放内存
            delete[] byteArray;
            
            if (bytesWritten != byteSize) {
                std::cout << "发送数据不完整，已发送 " << bytesWritten << " 字节，预期发送 " << byteSize << " 字节" << std::endl;
                return false;
            }
            
            return true;
        }
        catch (std::exception& e) {
            std::cout << "发送数据数组异常: " << e.what() << std::endl;
            return false;
        }
    }
    
    // 读取数据
    std::string receive(size_t size = 1024);
    
    // 读取数据到字节数组
    std::vector<uint8_t> receiveArray(size_t size = 1024);
    
    // 读取数据到缓冲区，返回读取的字节数
    int readBuffer(uint8_t* buffer, size_t size);
    
    // 关闭串口
    void close();
    
    // 检查串口是否已打开
    bool isOpen() const;
    
    // Add function to send Point data to 32-bit MCU
    bool sendPoint(int x, int y);

private:
    serial::Serial ser;
    std::string portName;
    uint32_t baudRate;
    bool isConnected;
};

#endif