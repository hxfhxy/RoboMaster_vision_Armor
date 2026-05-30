#include "cpp08_armor_detector/uart_work.hpp"

// 初始化串口
bool RoboMasterUART::init(const char* device, int baud) {
    fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) {
        std::cerr << "无法打开串口: " << device << "（可能被占用）" << std::endl;
        perror("open");
        return false;
    }

    struct termios options;
    tcgetattr(fd, &options);

    //闭终端回显、规范模式、输出处理
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 关闭规范模式/回显/信号
    options.c_oflag &= ~OPOST;                          // 关闭输出字节修改
    options.c_iflag &= ~(IXON | IXOFF | IXANY);         // 关闭流控

    // 波特率 + 8N1 配置（原有逻辑保留）
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // 非规范模式读取配置（提升稳定性）
    options.c_cc[VMIN] = 0;    // 读取不等待最小字节数
    options.c_cc[VTIME] = 10;  // 超时100ms（单位：0.1s）

    // 激活配置 + 清空缓冲区
    tcsetattr(fd, TCSANOW, &options);
    tcflush(fd, TCIOFLUSH);

    return true;
}


// 发送数据到电控 
bool RoboMasterUART::send(Manifold_UART_Rx_Data &data) {
    if (fd == -1) {
        std::cerr << "串口未打开，发送失败！" << std::endl;
        return false;
    }
    // 调用协议层的发送函数
    return (UART_SendData(fd, &data) == 0);
}

// 接收电控发的云台数据
bool RoboMasterUART::receiveGimbal(GimbalToVision_Data &out_data) {
    uint8_t tmp_buf[128];
    // 1. 从硬件读取原始数据
    ssize_t len = read(fd, tmp_buf, sizeof(tmp_buf));
    
    if (len > 0) {
        rx_buffer.insert(rx_buffer.end(), tmp_buf, tmp_buf + len);
    }

    // 2. 状态机解析（处理粘包、伪帧头）
    while (rx_buffer.size() >= sizeof(GimbalToVision_Data)) {
        if (rx_buffer[0] == 0xFE) { // 找帧头
            GimbalToVision_Data* p = reinterpret_cast<GimbalToVision_Data*>(rx_buffer.data());
            if (p->Frame_Tail == 0xFF) { // 校验帧尾
                out_data = *p; // 拷贝有效数据
                // 移除已处理的包
                rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + sizeof(GimbalToVision_Data));
                return true;
            } else {
                rx_buffer.erase(rx_buffer.begin()); // 伪帧头，丢弃
            }
        } else {
            rx_buffer.erase(rx_buffer.begin()); // 不是帧头，丢弃
        }
    }
    return false; // 没有有效数据
}

// 析构函数（关闭串口）
RoboMasterUART::~RoboMasterUART() {
    if (fd != -1) close(fd);
}
