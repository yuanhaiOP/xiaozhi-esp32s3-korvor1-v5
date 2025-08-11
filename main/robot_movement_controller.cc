#include "robot_movement_controller.h"
#include <algorithm>
#include <cstring>
#include <sstream>
#include <cmath>

RobotMovementController::RobotMovementController(uart_port_t uart_port, int tx_pin, int rx_pin, int baud_rate)
    : uart_port_(uart_port), tx_pin_(tx_pin), rx_pin_(rx_pin), baud_rate_(baud_rate) {
    InitializeCommandMap();
}

RobotMovementController::~RobotMovementController() {
    // UART驱动会在系统关闭时自动清理
}

void RobotMovementController::InitializeCommandMap() {
    // 前进指令
    command_map_["前进"] = MovementCommand::FORWARD;
    command_map_["向前"] = MovementCommand::FORWARD;
    command_map_["往前走"] = MovementCommand::FORWARD;
    command_map_["向前走"] = MovementCommand::FORWARD;
    command_map_["直走"] = MovementCommand::FORWARD;
    command_map_["go forward"] = MovementCommand::FORWARD;
    command_map_["forward"] = MovementCommand::FORWARD;
    command_map_["go ahead"] = MovementCommand::FORWARD;
    
    // 后退指令
    command_map_["后退"] = MovementCommand::BACKWARD;
    command_map_["向后"] = MovementCommand::BACKWARD;
    command_map_["往后走"] = MovementCommand::BACKWARD;
    command_map_["向后走"] = MovementCommand::BACKWARD;
    command_map_["go back"] = MovementCommand::BACKWARD;
    command_map_["backward"] = MovementCommand::BACKWARD;
    command_map_["back"] = MovementCommand::BACKWARD;
    
    // 左转指令
    command_map_["左转"] = MovementCommand::TURN_LEFT;
    command_map_["向左转"] = MovementCommand::TURN_LEFT;
    command_map_["向左"] = MovementCommand::TURN_LEFT;
    command_map_["turn left"] = MovementCommand::TURN_LEFT;
    command_map_["left"] = MovementCommand::TURN_LEFT;
    command_map_["go left"] = MovementCommand::TURN_LEFT;
    
    // 右转指令
    command_map_["右转"] = MovementCommand::TURN_RIGHT;
    command_map_["向右转"] = MovementCommand::TURN_RIGHT;
    command_map_["向右"] = MovementCommand::TURN_RIGHT;
    command_map_["turn right"] = MovementCommand::TURN_RIGHT;
    command_map_["right"] = MovementCommand::TURN_RIGHT;
    command_map_["go right"] = MovementCommand::TURN_RIGHT;
    
    // 停止指令
    command_map_["停止"] = MovementCommand::STOP;
    command_map_["停下"] = MovementCommand::STOP;
    command_map_["停"] = MovementCommand::STOP;
    command_map_["stop"] = MovementCommand::STOP;
    command_map_["halt"] = MovementCommand::STOP;
    
    // 跳舞指令
    command_map_["跳舞"] = MovementCommand::DANCE;
    command_map_["dance"] = MovementCommand::DANCE;
    command_map_["show dance"] = MovementCommand::DANCE;
    
    // 灯光指令
    command_map_["开灯"] = MovementCommand::LIGHT_ON;
    command_map_["打开灯"] = MovementCommand::LIGHT_ON;
    command_map_["亮灯"] = MovementCommand::LIGHT_ON;
    command_map_["turn on light"] = MovementCommand::LIGHT_ON;
    command_map_["light on"] = MovementCommand::LIGHT_ON;
    
    command_map_["关灯"] = MovementCommand::LIGHT_OFF;
    command_map_["关闭灯"] = MovementCommand::LIGHT_OFF;
    command_map_["灭灯"] = MovementCommand::LIGHT_OFF;
    command_map_["turn off light"] = MovementCommand::LIGHT_OFF;
    command_map_["light off"] = MovementCommand::LIGHT_OFF;
}

bool RobotMovementController::Initialize() {
    // 对于UART0，我们直接使用系统已经配置好的UART，不重新初始化
    if (uart_port_ == UART_NUM_0) {
        ESP_LOGI(TAG, "Using system-configured UART0 for robot control");
        ESP_LOGI(TAG, "Robot movement controller ready on UART0, Baud:%d", baud_rate_);
        return true;
    }
    
    // 对于其他UART端口，正常初始化
    uart_config_t uart_config = {
        .baud_rate = baud_rate_,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    esp_err_t ret = uart_driver_install(uart_port_, 1024, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = uart_param_config(uart_port_, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(ret));
        return false;
    }
    
    // 设置UART引脚
    ret = uart_set_pin(uart_port_, tx_pin_, rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "Robot movement controller initialized on UART%d, TX:%d, RX:%d, Baud:%d", 
             uart_port_, tx_pin_, rx_pin_, baud_rate_);
    return true;
}

// 简单的字符串查找函数
bool ContainsString(const std::string& text, const std::string& pattern) {
    return text.find(pattern) != std::string::npos;
}

// 简单的数字提取函数
float ExtractNumber(const std::string& text, const std::string& pattern) {
    size_t pos = text.find(pattern);
    if (pos == std::string::npos) return 0.0f;
    
    // 查找数字
    size_t start = pos + pattern.length();
    while (start < text.length() && !isdigit(text[start]) && text[start] != '.') {
        start++;
    }
    if (start >= text.length()) return 0.0f;
    
    size_t end = start;
    while (end < text.length() && (isdigit(text[end]) || text[end] == '.')) {
        end++;
    }
    
    return std::stof(text.substr(start, end - start));
}

MovementParams RobotMovementController::ParseVoiceCommand(const std::string& voice_command) {
    MovementParams params;
    std::string command_lower = voice_command;
    
    // 转换为小写
    std::transform(command_lower.begin(), command_lower.end(), command_lower.begin(), ::tolower);
    
    ESP_LOGI(TAG, "Parsing voice command: '%s'", voice_command.c_str());
    
    // 查找匹配的指令
    for (const auto& pair : command_map_) {
        std::string key_lower = pair.first;
        std::transform(key_lower.begin(), key_lower.end(), key_lower.begin(), ::tolower);
        
        if (ContainsString(command_lower, key_lower)) {
            params.command = pair.second;
            ESP_LOGI(TAG, "Matched command: %s", pair.first.c_str());
            break;
        }
    }
    
    // 解析目标位姿：前进/后退 N 米；左转/右转 N 度/弧度
    // 缺省距离与角度
    const float default_distance = 0.5f; // 米
    const float default_angle_rad = 3.1415926f / 6.0f; // 30°

    // 距离解析
    if (params.command == MovementCommand::FORWARD || params.command == MovementCommand::BACKWARD) {
        float value = ExtractNumber(command_lower, "");
        if (value <= 0.0f) value = default_distance;
        params.distance_meters = (params.command == MovementCommand::FORWARD) ? value : -value;
    }

    // 角度解析（优先度→弧度关键字，其次度数）
    if (params.command == MovementCommand::TURN_LEFT || params.command == MovementCommand::TURN_RIGHT) {
        float value = 0.0f;
        bool has_value = false;
        if (ContainsString(command_lower, "弧度") || ContainsString(command_lower, "rad")) {
            value = ExtractNumber(command_lower, "");
            has_value = value > 0.0f;
        }
        if (!has_value) {
            // 解析度数并转换为弧度
            value = ExtractNumber(command_lower, "");
            if (value <= 0.0f) value = 30.0f; // 默认30°
            value = value * 3.1415926f / 180.0f;
        }
        params.angle_radians = (params.command == MovementCommand::TURN_LEFT) ? +value : -value;
    }
    
    // 解析持续时间
    if (ContainsString(command_lower, "秒") || ContainsString(command_lower, "second") || ContainsString(command_lower, "s")) {
        params.duration = ExtractNumber(command_lower, "");
    }
    
    // 解析灯光模式
    if (ContainsString(command_lower, "号") || ContainsString(command_lower, "mode")) {
        float mode = ExtractNumber(command_lower, "");
        if (mode >= 1 && mode <= 6) {
            params.light_mode = static_cast<int>(mode);
        }
    }
    
    return params;
}

std::string RobotMovementController::GenerateMovementCommand(const MovementParams& params) {
    uint8_t frame[ROBOT_FRAME_LENGTH];
    
    switch (params.command) {
        case MovementCommand::FORWARD:
        case MovementCommand::BACKWARD:
            // x = distance (m), y = angle (rad)
            CreateMoveFrame(params.distance_meters, 0.0f, frame);
            break;
        case MovementCommand::TURN_LEFT:
        case MovementCommand::TURN_RIGHT:
            // x = distance (m), y = angle (rad)
            CreateMoveFrame(0.0f, params.angle_radians, frame);
            break;
        case MovementCommand::STOP:
            CreateStopFrame(frame);
            break;
        case MovementCommand::DANCE:
            CreateDanceFrame(frame);
            break;
        case MovementCommand::LIGHT_ON:
            CreateLightFrame(params.light_mode, frame);
            break;
        case MovementCommand::LIGHT_OFF:
            CreateLightFrame(0, frame);
            break;
        default:
            CreateStopFrame(frame);
            break;
    }
    
    // 返回十六进制字符串用于日志
    char hex_string[32];
    snprintf(hex_string, sizeof(hex_string), "%02X%02X%02X%02X%02X%02X%02X%02X",
             frame[0], frame[1], frame[2], frame[3], frame[4], frame[5], frame[6], frame[7]);
    
    return std::string(hex_string);
}

void RobotMovementController::SendUartCommand(const std::string& command) {
    ESP_LOGI(TAG, "Sending robot command: %s", command.c_str());
    
    // 对于UART0，我们使用printf直接输出到串口，这是最简单可靠的方法
    if (uart_port_ == UART_NUM_0) {
        // 使用printf直接输出到UART0，这是系统默认的日志输出方式
        printf("%s\n", command.c_str());
        ESP_LOGI(TAG, "Robot command sent via printf to UART0: %s", command.c_str());
        return;
    }
    
    // 对于其他UART端口，使用正常的UART发送
    esp_err_t ret = uart_write_bytes(uart_port_, command.c_str(), command.length());
    if (ret == ESP_FAIL) {
        ESP_LOGE(TAG, "Failed to send UART command");
    } else {
        ESP_LOGI(TAG, "UART command sent successfully, length: %d", ret);
    }
}

bool RobotMovementController::ProcessVoiceCommand(const std::string& voice_command) {
    MovementParams params = ParseVoiceCommand(voice_command);
    
    if (params.command == MovementCommand::UNKNOWN) {
        ESP_LOGW(TAG, "Unknown voice command: %s", voice_command.c_str());
        return false;
    }
    
    // 生成数据帧并发送
    uint8_t frame[ROBOT_FRAME_LENGTH];
    switch (params.command) {
        case MovementCommand::FORWARD:
        case MovementCommand::BACKWARD:
            CreateMoveFrame(0.0f, params.distance_meters, frame);
            break;
        case MovementCommand::TURN_LEFT:
        case MovementCommand::TURN_RIGHT:
            CreateMoveFrame(params.angle_radians, 0.0f, frame);
            break;
        case MovementCommand::STOP:
            CreateStopFrame(frame);
            break;
        case MovementCommand::DANCE:
            CreateDanceFrame(frame);
            break;
        case MovementCommand::LIGHT_ON:
            CreateLightFrame(params.light_mode, frame);
            break;
        case MovementCommand::LIGHT_OFF:
            CreateLightFrame(0, frame);
            break;
        default:
            CreateStopFrame(frame);
            break;
    }
    
    // 发送数据帧
    SendFrame(frame, ROBOT_FRAME_LENGTH);
    
    ESP_LOGI(TAG, "Processed voice command: '%s' (distance: %.3f m, angle: %.3f rad, duration: %.1f s)", 
             voice_command.c_str(), params.distance_meters, params.angle_radians, params.duration);
    
    return true;
}

bool RobotMovementController::SendMovementCommand(MovementCommand command, float linear_speed, float angular_speed, float duration) {
    MovementParams params;
    params.command = command;
    params.duration = duration;
    // 兼容现有接口参数：
    // 当为前进/后退时，使用 linear_speed 作为“距离(米)”；
    // 当为转向时，使用 angular_speed 作为“角度(弧度)”。
    if (command == MovementCommand::FORWARD) {
        params.distance_meters = std::fabs(linear_speed);
        params.angle_radians = 0.0f;
    } else if (command == MovementCommand::BACKWARD) {
        params.distance_meters = -std::fabs(linear_speed);
        params.angle_radians = 0.0f;
    } else if (command == MovementCommand::TURN_LEFT) {
        params.distance_meters = 0.0f;
        params.angle_radians = std::fabs(angular_speed);
    } else if (command == MovementCommand::TURN_RIGHT) {
        params.distance_meters = 0.0f;
        params.angle_radians = -std::fabs(angular_speed);
    }
    
    // 生成数据帧并发送
    uint8_t frame[ROBOT_FRAME_LENGTH];
    switch (params.command) {
        case MovementCommand::FORWARD:
            CreateMoveFrame(params.distance_meters, 0.0f, frame);
            break;
        case MovementCommand::BACKWARD:
            CreateMoveFrame(params.distance_meters, 0.0f, frame);
            break;
        case MovementCommand::TURN_LEFT:
            CreateMoveFrame(0.0f, params.angle_radians, frame);
            break;
        case MovementCommand::TURN_RIGHT:
            CreateMoveFrame(0.0f, params.angle_radians, frame);
            break;
        case MovementCommand::STOP:
            CreateStopFrame(frame);
            break;
        case MovementCommand::DANCE:
            CreateDanceFrame(frame);
            break;
        case MovementCommand::LIGHT_ON:
            CreateLightFrame(params.light_mode, frame);
            break;
        case MovementCommand::LIGHT_OFF:
            CreateLightFrame(0, frame);
            break;
        default:
            CreateStopFrame(frame);
            break;
    }
    
    // 发送数据帧
    SendFrame(frame, ROBOT_FRAME_LENGTH);
    
    return true;
}

bool RobotMovementController::SendLightCommand(int light_mode) {
    if (light_mode < 0 || light_mode > 6) {
        ESP_LOGE(TAG, "Invalid light mode: %d", light_mode);
        return false;
    }
    
    uint8_t frame[ROBOT_FRAME_LENGTH];
    CreateLightFrame(light_mode, frame);
    SendFrame(frame, ROBOT_FRAME_LENGTH);
    
    return true;
}

bool RobotMovementController::Stop() {
    return SendMovementCommand(MovementCommand::STOP);
}

// 计算校验和（异或校验）
uint8_t RobotMovementController::CalculateChecksum(const uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// 创建移动指令数据帧
void RobotMovementController::CreateMoveFrame(float x, float y, uint8_t* frame) {
    // 将浮点数转换为整数（放大1000倍）
    int16_t x_int = (int16_t)(x * ROBOT_FLOAT_SCALE);
    int16_t y_int = (int16_t)(y * ROBOT_FLOAT_SCALE);
    
    // 构建数据帧
    frame[0] = ROBOT_FRAME_HEADER;                    // 帧头 0xAA
    frame[1] = ROBOT_CMD_MOVE;                        // 命令类型 0x01
    frame[2] = (x_int >> 8) & 0xFF;                  // X高字节
    frame[3] = x_int & 0xFF;                         // X低字节
    frame[4] = (y_int >> 8) & 0xFF;                  // Y高字节
    frame[5] = y_int & 0xFF;                         // Y低字节
    
    // 计算校验和（不包括帧头）
    uint8_t checksum = CalculateChecksum(&frame[1], 5);
    frame[6] = checksum;                              // 校验和
    
    frame[7] = ROBOT_FRAME_TAIL;                      // 帧尾 0x55
    
    ESP_LOGI(TAG, "Created move frame: x=%.3f->%d, y=%.3f->%d", x, x_int, y, y_int);
}

// 创建跳舞指令数据帧
void RobotMovementController::CreateDanceFrame(uint8_t* frame) {
    frame[0] = ROBOT_FRAME_HEADER;                    // 帧头 0xAA
    frame[1] = ROBOT_CMD_DANCE;                       // 命令类型 0x02
    frame[2] = 0x00;                                  // 数据高字节（未使用）
    frame[3] = 0x00;                                  // 数据低字节（未使用）
    frame[4] = 0x00;                                  // 数据高字节（未使用）
    frame[5] = 0x00;                                  // 数据低字节（未使用）
    
    // 计算校验和
    uint8_t checksum = CalculateChecksum(&frame[1], 5);
    frame[6] = checksum;                              // 校验和
    
    frame[7] = ROBOT_FRAME_TAIL;                      // 帧尾 0x55
    
    ESP_LOGI(TAG, "Created dance frame");
}

// 创建灯光指令数据帧
void RobotMovementController::CreateLightFrame(int mode, uint8_t* frame) {
    frame[0] = ROBOT_FRAME_HEADER;                    // 帧头 0xAA
    frame[1] = ROBOT_CMD_LIGHT;                       // 命令类型 0x03
    frame[2] = 0x00;                                  // 数据高字节（未使用）
    frame[3] = 0x00;                                  // 数据低字节（未使用）
    frame[4] = 0x00;                                  // 数据高字节（未使用）
    frame[5] = (uint8_t)mode;                         // 灯光模式
    
    // 计算校验和
    uint8_t checksum = CalculateChecksum(&frame[1], 5);
    frame[6] = checksum;                              // 校验和
    
    frame[7] = ROBOT_FRAME_TAIL;                      // 帧尾 0x55
    
    ESP_LOGI(TAG, "Created light frame: mode=%d", mode);
}

// 创建停止指令数据帧
void RobotMovementController::CreateStopFrame(uint8_t* frame) {
    frame[0] = ROBOT_FRAME_HEADER;                    // 帧头 0xAA
    frame[1] = ROBOT_CMD_STOP;                        // 命令类型 0x04
    frame[2] = 0x00;                                  // 数据高字节（未使用）
    frame[3] = 0x00;                                  // 数据低字节（未使用）
    frame[4] = 0x00;                                  // 数据高字节（未使用）
    frame[5] = 0x00;                                  // 数据低字节（未使用）
    
    // 计算校验和
    uint8_t checksum = CalculateChecksum(&frame[1], 5);
    frame[6] = checksum;                              // 校验和
    
    frame[7] = ROBOT_FRAME_TAIL;                      // 帧尾 0x55
    
    ESP_LOGI(TAG, "Created stop frame");
}

// 发送数据帧
void RobotMovementController::SendFrame(const uint8_t* frame, size_t length) {
    ESP_LOGI(TAG, "Sending frame: ");
    for (size_t i = 0; i < length; i++) {
        printf("%02X ", frame[i]);
    }
    printf("\n");
}