#ifndef UART_PROTOCOL_HPP
#define UART_PROTOCOL_HPP
#include <stdint.h>
#include <stddef.h>

// 跨编译器禁用结构体填充（GCC/Clang/MSVC通用）
#if defined(__GNUC__)
#define PACKED __attribute__((packed))
#elif defined(_MSC_VER)
#define PACKED __pragma(pack(push,1)) __pragma(pack(pop))
#else
#define PACKED
#endif

// 敌方ID枚举
typedef enum {
    ENEMY_ID_UNKNOWN = 0,
    ENEMY_ID_HERO = 1,
    ENEMY_ID_ENGINEER = 2,
    ENEMY_ID_INFANTRY_3 = 3,
    ENEMY_ID_INFANTRY_4 = 4,
    ENEMY_ID_SENTRY = 5,
} Enum_Manifold_Enemy_ID;

//电控发视觉的数据包
typedef struct PACKED {
    uint8_t Frame_Header;          // 帧头：0xFE (1字节)
    float Gimbal_Yaw_Current;      // 云台当前yaw（°）(4字节)
    float Gimbal_Pitch_Current;    // 云台当前pitch（°）(4字节)
    uint8_t Frame_Tail;            // 帧尾：0xFF (1字节)
    // 总字节数：1+4+4+1=10字节
} GimbalToVision_Data;

//视觉发电控的数据包
typedef struct PACKED {
    uint8_t Frame_Header;          // 帧头：0xFE (1字节)
    float Gimbal_Yaw_Angle;        // 云台偏航角绝对量（°）(4字节)
    float Gimbal_Pitch_Angle;      // 云台俯仰角绝对值（°）(4字节)
    uint8_t target_valid;     // 目标有效位：0=无效，1=有效
    uint8_t Frame_Tail;            // 帧尾：0xFF (1字节)
    // 总字节数：1+4+4+1=10字节
} Manifold_UART_Rx_Data;

// 发送数据到电控
uint8_t UART_SendData(int uart_fd, Manifold_UART_Rx_Data *data);
// 解析电控发的视觉数据
uint8_t UART_ParseData(uint8_t *buf, size_t len, Manifold_UART_Rx_Data *data);
// 解析电控发的云台数据
uint8_t UART_ParseGimbalData(uint8_t *buf, size_t len, GimbalToVision_Data *data);

#endif // UART_PROTOCOL_HPP
