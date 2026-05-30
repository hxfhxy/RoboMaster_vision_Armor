#ifndef UART_WORKER_HPP
#define UART_WORKER_HPP
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include "cpp08_armor_detector/uart_protocol.hpp"

class RoboMasterUART {
private:
    int fd;
    std::vector<uint8_t> rx_buffer; // 接收缓冲区（处理粘包用）

public:
    RoboMasterUART() : fd(-1) {}
    
    // 初始化串口
    bool init(const char* device = "/dev/ttyACM0", int baud = B115200);
    
    // 发送数据到电控 
    bool send(Manifold_UART_Rx_Data &data);
    
    // 接收电控发的云台数据
    bool receiveGimbal(GimbalToVision_Data &out_data);
    
    //析构函数（关闭串口）
    ~RoboMasterUART();
};

#endif // UART_WORKER_HPP
