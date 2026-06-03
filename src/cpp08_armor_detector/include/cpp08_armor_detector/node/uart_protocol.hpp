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
enum Enum_Manifold_Enemy_ID : uint8_t
{
    Manifold_Enemy_ID_NONE_0 = 0,
    Manifold_Enemy_ID_HERO_1=1,
    Manifold_Enemy_ID_ENGINEER_2=2,
    Manifold_Enemy_ID_INFANTRY_3=3,
    Manifold_Enemy_ID_INFANTRY_4=4,
    Manifold_Enemy_ID_INFANTRY_5=5,
    Manifold_Enemy_ID_SENTRY_7=7,
    Manifold_Enemy_ID_OUTPOST=8,
    Manifold_Enemy_ID_RUNE=9,
};

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
    //uint8_t armor_number;          // 识别到的装甲板号码（0-9），255表示未识别到有效号码
    uint8_t Frame_Tail;            // 帧尾：0xFF (1字节)
    // 总字节数：1+4+4+1+1=11字节
} Manifold_UART_Rx_Data;

// 发送数据到电控
uint8_t UART_SendData(int uart_fd, Manifold_UART_Rx_Data *data);
// 解析电控发的视觉数据
uint8_t UART_ParseData(uint8_t *buf, size_t len, Manifold_UART_Rx_Data *data);
// 解析电控发的云台数据
uint8_t UART_ParseGimbalData(uint8_t *buf, size_t len, GimbalToVision_Data *data);

#endif // UART_PROTOCOL_HPP
