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
    std::vector<uint8_t> rx_buffer;

public:
    RoboMasterUART() : fd(-1) {}

    bool init(const char* device = "/dev/ttyUSB0", int baud = B115200) {
        fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd == -1) {
            std::cerr << "无法打开串口: " << device << std::endl;
            return false;
        }

        struct termios options;
        tcgetattr(fd, &options);
        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);

        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        
        tcsetattr(fd, TCSANOW, &options);
        return true;
    }

    // 补充发送函数供节点调用
    bool send(Manifold_UART_Rx_Data &data) {
        if (fd == -1) return false;
        // 调用你定义的全局发送函数
        return (UART_SendData(fd, &data) == 0);
    }

    bool receive(Manifold_UART_Rx_Data &out_data) {
        uint8_t tmp_buf[128];
        ssize_t len = read(fd, tmp_buf, sizeof(tmp_buf));
        if (len > 0) {
            rx_buffer.insert(rx_buffer.end(), tmp_buf, tmp_buf + len);
        }

        while (rx_buffer.size() >= sizeof(Manifold_UART_Rx_Data)) {
            if (rx_buffer[0] == 0xFF) {
                Manifold_UART_Rx_Data* p = reinterpret_cast<Manifold_UART_Rx_Data*>(rx_buffer.data());
                if (p->Frame_Tail == 0xFE) {
                    out_data = *p;
                    rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + sizeof(Manifold_UART_Rx_Data));
                    return true;
                } else {
                    rx_buffer.erase(rx_buffer.begin());
                }
            } else {
                rx_buffer.erase(rx_buffer.begin());
            }
        }
        return false;
    }

    ~RoboMasterUART() {
        if (fd != -1) close(fd);
    }
};

#endif