#ifndef UART_PROTOCOL_HPP
#define UART_PROTOCOL_HPP

#include <stdint.h>
#include <stddef.h>

// 敌方ID
typedef enum {
    ENEMY_ID_UNKNOWN = 0,
    ENEMY_ID_HERO = 1,
    ENEMY_ID_ENGINEER = 2,
    ENEMY_ID_INFANTRY_3 = 3,
    ENEMY_ID_INFANTRY_4 = 4,
    ENEMY_ID_SENTRY = 5,
} Enum_Manifold_Enemy_ID;

// 串口数据结构体（packed 禁止对齐）
typedef struct __attribute__((packed)) {
    uint8_t Frame_Header;          // 帧头：0xFF
    uint8_t Frame_Tail;            // 帧尾：0xFE
    //uint8_t Shoot_Flag;            // 0=不射击，1=射击
    float Gimbal_Pitch_Angle;      // 云台俯仰角绝对值
    float Gimbal_Yaw_Angle_Increment; // 云台偏航角增量
    Enum_Manifold_Enemy_ID Enemy_ID;   // 敌方机器人ID
} Manifold_UART_Rx_Data;

// 串口操作函数声明
uint8_t UART_SendData(int uart_fd, Manifold_UART_Rx_Data *data);
uint8_t UART_ParseData(uint8_t *buf, size_t len, Manifold_UART_Rx_Data *data);

#endif // UART_PROTOCOL_HPP
