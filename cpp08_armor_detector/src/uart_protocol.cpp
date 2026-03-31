#include "cpp08_armor_detector/uart_protocol.hpp"
#include <string.h>
#include <unistd.h>

// 发送数据到串口
uint8_t UART_SendData(int uart_fd, Manifold_UART_Rx_Data *data) {
    if (uart_fd < 0 || data == NULL) return 1;

    // 强制填充帧头帧尾
    data->Frame_Header = 0xFF;
    data->Frame_Tail   = 0xFE;

    // 直接发送结构体（packed 保证字节序正确）
    ssize_t bytes_written = write(uart_fd, data, sizeof(Manifold_UART_Rx_Data));
    return (bytes_written == sizeof(Manifold_UART_Rx_Data)) ? 0 : 1;
}

// 解析串口数据
uint8_t UART_ParseData(uint8_t *buf, size_t len, Manifold_UART_Rx_Data *data) {
    if (buf == NULL || data == NULL || len < sizeof(Manifold_UART_Rx_Data)) {
        return 1;
    }

    // 拷贝到结构体
    memcpy(data, buf, sizeof(Manifold_UART_Rx_Data));

    // 校验帧头帧尾
    if (data->Frame_Header != 0xFF || data->Frame_Tail != 0xFE) {
        return 1;
    }

    return 0;
}
