#ifndef ROBOT_MOVEMENT_CONTROLLER_H
#define ROBOT_MOVEMENT_CONTROLLER_H

#include <string>
#include <map>
#include <vector>
#include <functional>
#include <driver/uart.h>
#include <esp_log.h>
#include "robot_config.h"

#define TAG "RobotController"

// 机器人移动指令类型
enum class MovementCommand {
    FORWARD,        // 前进
    BACKWARD,       // 后退
    TURN_LEFT,      // 左转
    TURN_RIGHT,     // 右转
    STOP,           // 停止
    DANCE,          // 跳舞
    LIGHT_ON,       // 开灯
    LIGHT_OFF,      // 关灯
    UNKNOWN         // 未知指令
};

// 移动参数结构
struct MovementParams {
    MovementCommand command;
    // 目标位姿增量
    float distance_meters;  // 目标直线位移 (米，正=前进，负=后退)
    float angle_radians;    // 目标转角 (弧度，正=左转，负=右转)
    float duration;         // 可选：持续时间 (秒)，当前协议不需要
    int light_mode;         // 灯光模式 (1-6)

    MovementParams()
        : command(MovementCommand::UNKNOWN),
          distance_meters(0.0f),
          angle_radians(0.0f),
          duration(0.0f),
          light_mode(1) {}
};

class RobotMovementController {
private:
    uart_port_t uart_port_;
    int tx_pin_;
    int rx_pin_;
    int baud_rate_;
    
    // 语音指令映射表
    std::map<std::string, MovementCommand> command_map_;
    
    // 初始化语音指令映射
    void InitializeCommandMap();
    
    // 解析语音指令
    MovementParams ParseVoiceCommand(const std::string& voice_command);
    
    // 解析中文数字的辅助函数
    float ParseChineseNumber(const std::string& chinese_text);
    
    // 检查是否是中文数字的辅助函数
    bool IsChineseNumber(const std::string& utf8_char);
    
    // 提取数字的辅助函数（包括阿拉伯数字和中文数字）
    float ExtractNumber(const std::string& text, const std::string& pattern);
    
    // 字符串包含检查的辅助函数
    bool ContainsString(const std::string& text, const std::string& pattern);
    
    // 发送UART命令
    void SendUartCommand(const std::string& command);
    
    // 生成移动命令字符串
    std::string GenerateMovementCommand(const MovementParams& params);

public:
    RobotMovementController(uart_port_t uart_port = UART_NUM_0, 
                           int tx_pin = 43, 
                           int rx_pin = 44, 
                           int baud_rate = 115200);
    ~RobotMovementController();
    
    // 初始化UART
    bool Initialize();
    
    // 处理语音指令
    bool ProcessVoiceCommand(const std::string& voice_command);
    
    // 直接发送移动命令
    bool SendMovementCommand(MovementCommand command, float linear_speed = 0.0, float angular_speed = 0.0, float duration = 0.0);
    
    // 发送灯光控制命令
    bool SendLightCommand(int light_mode);
    
    // 停止所有移动
    bool Stop();
    
    // 数据帧相关方法
    uint8_t CalculateChecksum(const uint8_t* data, size_t length);
    void CreateMoveFrame(float x, float y, uint8_t* frame);
    void CreateDanceFrame(uint8_t* frame);
    void CreateLightFrame(int mode, uint8_t* frame);
    void CreateStopFrame(uint8_t* frame);
    void SendFrame(const uint8_t* frame, size_t length);
};

#endif // ROBOT_MOVEMENT_CONTROLLER_H 