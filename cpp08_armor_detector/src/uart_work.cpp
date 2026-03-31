#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include "cpp08_armor_detector/uart_protocol.hpp"

class RoboMasterUART {
private:
    int fd;
    std::vector<uint8_t> rx_buffer; // 接收缓冲区，用于处理粘包

public:
    RoboMasterUART() : fd(-1) {}

    // 初始化串口
    bool init(const char* device = "/dev/ttyUSB0", int baud = B115200) {
        // O_RDWR: 读写模式 | O_NOCTTY: 不作为控制终端 | O_NONBLOCK: 非阻塞模式
        fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd == -1) {
            std::cerr << "无法打开串口: " << device << std::endl;
            return false;
        }

        struct termios options;
        tcgetattr(fd, &options);

        // 设置波特率
        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);

        // 典型配置：8N1 (8位数据, 无校验, 1位停止位)
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        
        // 激活配置
        tcsetattr(fd, TCSANOW, &options);
        return true;
    }

    // 核心读取函数
    bool receive(Manifold_UART_Rx_Data &out_data) {
        uint8_t tmp_buf[128];
        // 1. 从硬件读取原始数据
        ssize_t len = read(fd, tmp_buf, sizeof(tmp_buf));
        
        if (len > 0) {
            // 将新读到的数据存入 vector 缓冲区
            rx_buffer.insert(rx_buffer.end(), tmp_buf, tmp_buf + len);
        }

        // 2. 状态机解析：循环检查缓冲区
        // 确保长度至少够一个包
        while (rx_buffer.size() >= sizeof(Manifold_UART_Rx_Data)) {
            // 找帧头 0xFF
            if (rx_buffer[0] == 0xFF) {
                // 预解析（不弹出数据，只检查）
                Manifold_UART_Rx_Data* p = reinterpret_cast<Manifold_UART_Rx_Data*>(rx_buffer.data());
                
                // 校验帧尾 0xFE
                if (p->Frame_Tail == 0xFE) {
                    out_data = *p; // 拷贝有效数据
                    // 从缓冲区移除已处理的这一个包
                    rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + sizeof(Manifold_UART_Rx_Data));
                    return true; // 成功解析一帧
                } else {
                    // 帧头对了但帧尾不对，说明这个 0xFF 是数据里的伪帧头，弹出它继续找
                    rx_buffer.erase(rx_buffer.begin());
                }
            } else {
                // 不是帧头，直接丢弃该字节
                rx_buffer.erase(rx_buffer.begin());
            }
        }
        return false;
    }

    // 在 RoboMasterUART 类的 public 区域加入这个发送函数
    bool send(Manifold_UART_Rx_Data &data) {
        if (fd == -1) {
            std::cerr << "串口未打开，发送失败！" << std::endl;
            return false;
        }
        // 直接调用 uart_protocol.cpp 中的发送逻辑
        uint8_t ret = UART_SendData(fd, &data);
        return (ret == 0); // 返回 0 表示成功
    }

    ~RoboMasterUART() {
        if (fd != -1) close(fd);
    }
};