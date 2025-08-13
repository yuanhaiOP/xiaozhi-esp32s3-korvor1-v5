#include "robot_movement_controller.h"
#include <algorithm>
#include <cstring>
#include <sstream>
#include <cmath>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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
    
    // 向后转指令（转180度）
    command_map_["向后转"] = MovementCommand::TURN_LEFT;
    command_map_["向后转"] = MovementCommand::TURN_LEFT;
    command_map_["向后转"] = MovementCommand::TURN_LEFT;
    command_map_["turn around"] = MovementCommand::TURN_LEFT;
    command_map_["turn back"] = MovementCommand::TURN_LEFT;
    
    // 转圈指令
    command_map_["转一圈"] = MovementCommand::TURN_LEFT;
    command_map_["转两圈"] = MovementCommand::TURN_LEFT;
    command_map_["转三圈"] = MovementCommand::TURN_LEFT;
    command_map_["turn one circle"] = MovementCommand::TURN_LEFT;
    command_map_["turn two circles"] = MovementCommand::TURN_LEFT;
    command_map_["turn three circles"] = MovementCommand::TURN_LEFT;
    
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
    ESP_LOGI(TAG, "Initializing UART%d for robot control", uart_port_);
    
    if (uart_port_ == UART_NUM_0) {
        // 对于UART0，我们需要确保它能正确工作
        ESP_LOGI(TAG, "Initializing UART0 for robot control");
        
        // 检查UART0是否已经安装，并删除以重新安装
        uart_driver_delete(uart_port_);
        
        // 配置UART0参数
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
            ESP_LOGE(TAG, "Failed to install UART0 driver: %s", esp_err_to_name(ret));
            return false;
        }
        
        ret = uart_param_config(uart_port_, &uart_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure UART0: %s", esp_err_to_name(ret));
            return false;
        }
        
        // 对于UART0，使用默认引脚，不重新设置
        ESP_LOGI(TAG, "Robot movement controller ready on UART0, Baud:%d", baud_rate_);
        
        // 发送一个"激活"字节来确保UART0正常工作
        uint8_t activate_byte = 0x00;
        uart_write_bytes(uart_port_, &activate_byte, 1);
        vTaskDelay(pdMS_TO_TICKS(10)); // 等待UART0稳定
        
        return true;
    }
    
    // 对于其他UART端口，使用正常的UART配置
    uart_config_t uart_config = {
        .baud_rate = baud_rate_,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // 安装UART驱动
    esp_err_t ret = uart_driver_install(uart_port_, 1024, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return false;
    }
    
    // 配置UART参数
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
    
    ESP_LOGI(TAG, "Robot movement controller ready on UART%d, TX:%d, RX:%d, Baud:%d", 
             uart_port_, tx_pin_, rx_pin_, baud_rate_);
    return true;
}

// 简单的字符串查找函数
bool ContainsString(const std::string& text, const std::string& pattern) {
    return text.find(pattern) != std::string::npos;
}

// 简单的数字提取函数
float ExtractNumber(const std::string& text, const std::string& pattern) {
    size_t pos;
    if (pattern.empty()) {
        // 如果pattern为空，在整个字符串中查找数字
        pos = 0;
    } else {
        pos = text.find(pattern);
        if (pos == std::string::npos) return 0.0f;
    }
    
    // 查找数字
    size_t start = pos;
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
    
    // 简化的参数解析：只解析圈数
    if (params.command == MovementCommand::TURN_LEFT || params.command == MovementCommand::TURN_RIGHT) {
        // 检查是否有"圈"、"圈数"、"turn"等关键词
        if (ContainsString(command_lower, "圈") || ContainsString(command_lower, "turn")) {
            float circles = ExtractNumber(command_lower, "");
            if (circles > 0.0f) {
                // 一圈 = 2π弧度
                params.angle_radians = circles * 2.0f * 3.1415926f;
            } else {
                // 默认转90度
                params.angle_radians = 3.1415926f / 2.0f;
            }
        } else if (ContainsString(command_lower, "向后转") || ContainsString(command_lower, "turn around") || 
                   ContainsString(command_lower, "turn back")) {
            // 向后转 = 180度 = π弧度
            params.angle_radians = 3.1415926f;
        } else {
            // 默认转90度
            params.angle_radians = 3.1415926f / 2.0f;
        }
        
        // 左转为正角度，右转为负角度
        if (params.command == MovementCommand::TURN_RIGHT) {
            params.angle_radians = -params.angle_radians;
        }
    }
    
    // 前进/后退的距离解析
    if (params.command == MovementCommand::FORWARD || params.command == MovementCommand::BACKWARD) {
        // 检查是否有距离关键词
        if (ContainsString(command_lower, "米") || ContainsString(command_lower, "m") || 
            ContainsString(command_lower, "米") || ContainsString(command_lower, "meter")) {
            float distance = ExtractNumber(command_lower, "");
            if (distance > 0.0f) {
                params.distance_meters = distance;
            } else {
                params.distance_meters = 0.5f; // 默认0.5米
            }
        } else {
            params.distance_meters = 0.5f; // 默认0.5米
        }
        
        // 后退为负距离
        if (params.command == MovementCommand::BACKWARD) {
            params.distance_meters = -params.distance_meters;
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
    ESP_LOGI(TAG, "=== ProcessVoiceCommand called with: '%s' ===", voice_command.c_str());
    
    // 测试串口是否工作
    // printf("=== UART TEST: ProcessVoiceCommand called ===\n");
    // printf("Voice command: %s\n", voice_command.c_str());
    
    MovementParams params = ParseVoiceCommand(voice_command);
    if (params.command == MovementCommand::UNKNOWN) {
        ESP_LOGW(TAG, "Unknown voice command: %s", voice_command.c_str());
        return false;
    }
    
    // 生成x[位置] y[角度]格式的命令字符串用于日志
    char command_buffer[64];
    snprintf(command_buffer, sizeof(command_buffer), "x%.3f y%.3f", 
             params.distance_meters, params.angle_radians);
    
    ESP_LOGI(TAG, "Processed voice command: '%s' -> %s", 
             voice_command.c_str(), command_buffer);
    
    // 生成数据帧并发送（使用原有的数据帧机制）
    uint8_t frame[ROBOT_FRAME_LENGTH];
    switch (params.command) {
        case MovementCommand::FORWARD:
        case MovementCommand::BACKWARD:
            CreateMoveFrame(params.distance_meters, 0.0f, frame);
            break;
        case MovementCommand::TURN_LEFT:
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
    
    // 等待一小段时间确保串口发送完成
    vTaskDelay(pdMS_TO_TICKS(20));
    
    ESP_LOGI(TAG, "Voice command execution completed: %s", voice_command.c_str());
    
    return true;
}

bool RobotMovementController::SendMovementCommand(MovementCommand command, float linear_speed, float angular_speed, float duration) {
    MovementParams params;
    params.command = command;
    params.duration = duration;
    // 兼容现有接口参数：
    // 当为前进/后退时，使用 linear_speed 作为"距离(米)"；
    // 当为转向时，使用 angular_speed 作为"角度(弧度)"。
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
    uint8_t heartbeat = 0xFF;
    uart_write_bytes(uart_port_, &heartbeat, 1);
    vTaskDelay(pdMS_TO_TICKS(1)); // 短暂延迟
    ESP_LOGI(TAG, "Sending frame: ");
    for (size_t i = 0; i < length; i++) {
        printf("%02X ", frame[i]);
    }
    printf("\n");
}