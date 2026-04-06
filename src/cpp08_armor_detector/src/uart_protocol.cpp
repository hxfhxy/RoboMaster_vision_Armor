#include "cpp08_armor_detector/uart_protocol.hpp"
#include <string.h>
#include <unistd.h>

// 修复：发送数据到电控（强制填充正确的帧头帧尾，保证发送顺序）
uint8_t UART_SendData(int uart_fd, Manifold_UART_Rx_Data *data) {
    if (uart_fd < 0 || data == NULL) return 1;
    // 强制填充协议定义的帧头/帧尾（覆盖业务层错误赋值）
    data->Frame_Header = 0xFE;
    data->Frame_Tail   = 0xFF;
    // 发送结构体（按内存顺序发送：Frame_Header → Gimbal_Pitch_Angle → Gimbal_Yaw_Angle → Frame_Tail）
    ssize_t bytes_written = write(uart_fd, data, sizeof(Manifold_UART_Rx_Data));
    // 返回0=成功，1=失败
    return (bytes_written == sizeof(Manifold_UART_Rx_Data)) ? 0 : 1;
}

// 修复：解析视觉发电控数据（帧头0xFE，帧尾0xFF）
uint8_t UART_ParseData(uint8_t *buf, size_t len, Manifold_UART_Rx_Data *data) {
    if (buf == NULL || data == NULL || len < sizeof(Manifold_UART_Rx_Data)) return 1;
    memcpy(data, buf, sizeof(Manifold_UART_Rx_Data));
    // 校验帧头/帧尾（修正写反的问题）
    if (data->Frame_Header != 0xFE || data->Frame_Tail != 0xFF) return 1;
    return 0;
}   

// 修复：解析电控发的云台数据（帧头0xFE，帧尾0xFF）
uint8_t UART_ParseGimbalData(uint8_t *buf, size_t len, GimbalToVision_Data *data) {
    const size_t PACKET_LEN = sizeof(GimbalToVision_Data);
    if (buf == NULL || data == NULL || len < PACKET_LEN) return 1;
    
    // 逐字节拷贝（避免对齐问题）
    memcpy(data, buf, PACKET_LEN);
    
    // 双重校验：帧头+帧尾
    if (data->Frame_Header != 0xFE || data->Frame_Tail != 0xFF) {
        return 1; // 校验失败
    }
    
    // （可选）如果电控是小端/大端，这里加字节序转换（比如float的字节序）
    // 示例：假设电控是小端，视觉端是大端，需转换float
    // data->Gimbal_Yaw_Current = swap_float_endian(data->Gimbal_Yaw_Current);
    return 0;
}
