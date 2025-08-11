#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "driver/uart.h"

// 机器人UART配置 - 使用系统已配置的UART0
#define ROBOT_UART_PORT        UART_NUM_0
#define ROBOT_UART_TX_PIN      UART_PIN_NO_CHANGE  // 使用系统默认引脚
#define ROBOT_UART_RX_PIN      UART_PIN_NO_CHANGE  // 使用系统默认引脚
#define ROBOT_UART_BAUD_RATE   115200

// 机器人数据帧协议定义
#define ROBOT_FRAME_HEADER     0xAA    // 帧头
#define ROBOT_FRAME_TAIL       0x55    // 帧尾
#define ROBOT_CMD_MOVE         0x01    // 移动指令
#define ROBOT_CMD_DANCE        0x02    // 跳舞指令
#define ROBOT_CMD_LIGHT        0x03    // 灯光指令
#define ROBOT_CMD_STOP         0x04    // 停止指令

// 数据帧长度定义（单位：字节）
#define ROBOT_FRAME_LENGTH     8       // 总帧长度：1(头)+1(CMD)+2(X)+2(Y)+1(CHK)+1(尾) = 8
#define ROBOT_HEADER_LENGTH    1       // 帧头长度
#define ROBOT_CMD_LENGTH       1       // 命令长度
#define ROBOT_DATA_LENGTH      4       // 数据长度（2字节x + 2字节y）
#define ROBOT_CHECKSUM_LENGTH  1       // 校验长度
#define ROBOT_TAIL_LENGTH      1       // 帧尾长度

// 浮点数转换比例（放大1000倍，保留3位小数）
#define ROBOT_FLOAT_SCALE      1000.0f

// 默认速度参数
#define ROBOT_DEFAULT_LINEAR_SPEED    0.2f    // 默认线速度 0.2 m/s
#define ROBOT_DEFAULT_ANGULAR_SPEED   0.5f    // 默认角速度 0.5 rad/s
#define ROBOT_SLOW_LINEAR_SPEED       0.1f    // 慢速线速度 0.1 m/s
#define ROBOT_SLOW_ANGULAR_SPEED      0.3f    // 慢速角速度 0.3 rad/s
#define ROBOT_FAST_LINEAR_SPEED       0.4f    // 快速线速度 0.4 m/s
#define ROBOT_FAST_ANGULAR_SPEED      1.0f    // 快速角速度 1.0 rad/s

// 机器人状态
#define ROBOT_STATUS_IDLE      0
#define ROBOT_STATUS_MOVING    1
#define ROBOT_STATUS_DANCING   2

// 灯光模式
#define ROBOT_LIGHT_OFF        0
#define ROBOT_LIGHT_MODE_1     1
#define ROBOT_LIGHT_MODE_2     2
#define ROBOT_LIGHT_MODE_3     3
#define ROBOT_LIGHT_MODE_4     4
#define ROBOT_LIGHT_MODE_5     5
#define ROBOT_LIGHT_MODE_6     6

#endif // ROBOT_CONFIG_H 