/*
 * MCP Server Implementation
 * Reference: https://modelcontextprotocol.io/specification/2024-11-05
 */

#include "mcp_server.h"
#include <esp_log.h>
#include <esp_app_desc.h>
#include <algorithm>
#include <cstring>
#include <esp_pthread.h>

#include "application.h"
#include "display.h"
#include "board.h"
#include "robot_movement_controller.h"

#define DEFAULT_TOOLCALL_STACK_SIZE 6144

McpServer::McpServer() {
}

McpServer::~McpServer() {
    for (auto tool : tools_) {
        delete tool;
    }
    tools_.clear();
}

void McpServer::AddCommonTools() {
    // To speed up the response time, we add the common tools to the beginning of
    // the tools list to utilize the prompt cache.
    // Backup the original tools list and restore it after adding the common tools.
    auto original_tools = std::move(tools_);
    auto& board = Board::GetInstance();

    AddTool("self.get_device_status",
        "Provides the real-time information of the device, including the current status of the audio speaker, screen, battery, network, etc.\n"
        "Use this tool for: \n"
        "1. Answering questions about current condition (e.g. what is the current volume of the audio speaker?)\n"
        "2. As the first step to control the device (e.g. turn up / down the volume of the audio speaker, etc.)",
        PropertyList(),
        [&board](const PropertyList& properties) -> ReturnValue {
            return board.GetDeviceStatusJson();
        });

    AddTool("self.audio_speaker.set_volume", 
        "Set the volume of the audio speaker. If the current volume is unknown, you must call `self.get_device_status` tool first and then call this tool.",
        PropertyList({
            Property("volume", kPropertyTypeInteger, 0, 100)
        }), 
        [&board](const PropertyList& properties) -> ReturnValue {
            auto codec = board.GetAudioCodec();
            codec->SetOutputVolume(properties["volume"].value<int>());
            return true;
        });
    
    auto backlight = board.GetBacklight();
    if (backlight) {
        AddTool("self.screen.set_brightness",
            "Set the brightness of the screen.",
            PropertyList({
                Property("brightness", kPropertyTypeInteger, 0, 100)
            }),
            [backlight](const PropertyList& properties) -> ReturnValue {
                uint8_t brightness = static_cast<uint8_t>(properties["brightness"].value<int>());
                backlight->SetBrightness(brightness, true);
                return true;
            });
    }

    auto display = board.GetDisplay();
    if (display && !display->GetTheme().empty()) {
        AddTool("self.screen.set_theme",
            "Set the theme of the screen. The theme can be `light` or `dark`.",
            PropertyList({
                Property("theme", kPropertyTypeString)
            }),
            [display](const PropertyList& properties) -> ReturnValue {
                display->SetTheme(properties["theme"].value<std::string>().c_str());
                return true;
            });
    }

    auto camera = board.GetCamera();
    if (camera) {
        AddTool("self.camera.take_photo",
            "Take a photo and explain it. Use this tool after the user asks you to see something.\n"
            "Args:\n"
            "  `question`: The question that you want to ask about the photo.\n"
            "Return:\n"
            "  A JSON object that provides the photo information.",
            PropertyList({
                Property("question", kPropertyTypeString)
            }),
            [camera](const PropertyList& properties) -> ReturnValue {
                if (!camera->Capture()) {
                    return "{\"success\": false, \"message\": \"Failed to capture photo\"}";
                }
                auto question = properties["question"].value<std::string>();
                return camera->Explain(question);
            });
    }

    // 添加机器人移动控制工具
    static RobotMovementController robot_controller(ROBOT_UART_PORT, ROBOT_UART_TX_PIN, ROBOT_UART_RX_PIN, ROBOT_UART_BAUD_RATE);
    if (robot_controller.Initialize()) {
        AddTool("self.robot.control_by_voice",
            "Control robot movement through voice commands. This tool parses natural language voice commands and converts them to precise robot control instructions.\n"
            "Supported commands:\n"
            "  - Movement: '前进', '后退', '左转', '右转', '停止', 'go forward', 'turn left', etc.\n"
            "  - Speed: '慢', '快', '中速', 'slow', 'fast', 'medium'\n"
            "  - Duration: '3秒', '5 seconds'\n"
            "  - Light: '开灯', '关灯', '3号灯', 'turn on light'\n"
            "  - Special: '跳舞', 'dance'\n"
            "Examples:\n"
            "  - '前进' -> robot moves forward\n"
            "  - '快一点左转' -> robot turns left quickly\n"
            "  - '慢速前进3秒' -> robot moves forward slowly for 3 seconds\n"
            "  - '打开3号灯' -> turns on light mode 3",
            PropertyList({
                Property("voice_command", kPropertyTypeString)
            }),
            [](const PropertyList& properties) -> ReturnValue {
                std::string voice_command = properties["voice_command"].value<std::string>();
                bool success = robot_controller.ProcessVoiceCommand(voice_command);
                if (success) {
                    return "{\"success\": true, \"message\": \"Robot command executed successfully\", \"command\": \"" + voice_command + "\"}";
                } else {
                    return "{\"success\": false, \"message\": \"Unknown or invalid robot command\", \"command\": \"" + voice_command + "\"}";
                }
            });

        AddTool("self.robot.move",
            "Direct robot movement control with linear and angular velocity.\n"
            "Args:\n"
            "  `direction`: Movement direction ('forward', 'backward', 'left', 'right', 'stop')\n"
            "  `linear_speed`: Linear velocity in m/s (0.0 to 1.0, default 0.2)\n"
            "  `angular_speed`: Angular velocity in rad/s (0.0 to 2.0, default 0.5)\n"
            "  `duration`: Movement duration in seconds (0.0 for continuous, default 0.0)",
            PropertyList({
                Property("direction", kPropertyTypeString),
                Property("linear_speed", kPropertyTypeInteger, 20, 0, 100),
                Property("angular_speed", kPropertyTypeInteger, 50, 0, 200),
                Property("duration", kPropertyTypeInteger, 0, 0, 60)
            }),
            [](const PropertyList& properties) -> ReturnValue {
                std::string direction = properties["direction"].value<std::string>();
                float linear_speed = properties["linear_speed"].value<int>() / 100.0f;
                float angular_speed = properties["angular_speed"].value<int>() / 100.0f;
                float duration = properties["duration"].value<int>();
                
                MovementCommand command = MovementCommand::UNKNOWN;
                if (direction == "forward") command = MovementCommand::FORWARD;
                else if (direction == "backward") command = MovementCommand::BACKWARD;
                else if (direction == "left") command = MovementCommand::TURN_LEFT;
                else if (direction == "right") command = MovementCommand::TURN_RIGHT;
                else if (direction == "stop") command = MovementCommand::STOP;
                
                if (command == MovementCommand::UNKNOWN) {
                    return "{\"success\": false, \"message\": \"Invalid direction: " + direction + "\"}";
                }
                
                bool success = robot_controller.SendMovementCommand(command, linear_speed, angular_speed, duration);
                return success ? "{\"success\": true, \"message\": \"Movement command sent\"}" : "{\"success\": false, \"message\": \"Failed to send movement command\"}";
            });

        AddTool("self.robot.light",
            "Control robot light effects.\n"
            "Args:\n"
            "  `mode`: Light mode (0=off, 1-6=light effects)",
            PropertyList({
                Property("mode", kPropertyTypeInteger, 1, 0, 6)
            }),
            [](const PropertyList& properties) -> ReturnValue {
                int mode = properties["mode"].value<int>();
                bool success = robot_controller.SendLightCommand(mode);
                return success ? "{\"success\": true, \"message\": \"Light command sent\"}" : "{\"success\": false, \"message\": \"Failed to send light command\"}";
            });

        AddTool("self.robot.set_velocity",
            "Set robot linear and angular velocity directly.\n"
            "Args:\n"
            "  `linear_velocity`: Linear velocity in m/s (-1.0 to 1.0)\n"
            "  `angular_velocity`: Angular velocity in rad/s (-2.0 to 2.0)",
            PropertyList({
                Property("linear_velocity", kPropertyTypeInteger, 0, -100, 100),
                Property("angular_velocity", kPropertyTypeInteger, 0, -200, 200)
            }),
            [](const PropertyList& properties) -> ReturnValue {
                float linear_vel = properties["linear_velocity"].value<int>() / 100.0f;
                float angular_vel = properties["angular_velocity"].value<int>() / 100.0f;
                
                char buffer[32];
                snprintf(buffer, sizeof(buffer), "x%.3f y%.3f", angular_vel, linear_vel);
                std::string command = buffer;
                
                // 直接发送UART命令
                esp_err_t ret = uart_write_bytes(UART_NUM_0, command.c_str(), command.length());
                if (ret == ESP_FAIL) {
                    return "{\"success\": false, \"message\": \"Failed to send velocity command\"}";
                }
                
                ESP_LOGI("RobotController", "Sent velocity command: %s (linear: %.3f m/s, angular: %.3f rad/s)", 
                         command.c_str(), linear_vel, angular_vel);
                
                return "{\"success\": true, \"message\": \"Velocity command sent\", \"command\": \"" + command + "\"}";
            });

        AddTool("self.robot.stop",
            "Stop all robot movement immediately.",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                bool success = robot_controller.Stop();
                return success ? "{\"success\": true, \"message\": \"Robot stopped\"}" : "{\"success\": false, \"message\": \"Failed to stop robot\"}";
            });
    }

    // Restore the original tools list to the end of the tools list
    tools_.insert(tools_.end(), original_tools.begin(), original_tools.end());
}

void McpServer::AddTool(McpTool* tool) {
    // Prevent adding duplicate tools
    if (std::find_if(tools_.begin(), tools_.end(), [tool](const McpTool* t) { return t->name() == tool->name(); }) != tools_.end()) {
        ESP_LOGW(TAG, "Tool %s already added", tool->name().c_str());
        return;
    }

    ESP_LOGI(TAG, "Add tool: %s", tool->name().c_str());
    tools_.push_back(tool);
}

void McpServer::AddTool(const std::string& name, const std::string& description, const PropertyList& properties, std::function<ReturnValue(const PropertyList&)> callback) {
    AddTool(new McpTool(name, description, properties, callback));
}

void McpServer::ParseMessage(const std::string& message) {
    cJSON* json = cJSON_Parse(message.c_str());
    if (json == nullptr) {
        ESP_LOGE(TAG, "Failed to parse MCP message: %s", message.c_str());
        return;
    }
    ParseMessage(json);
    cJSON_Delete(json);
}

void McpServer::ParseCapabilities(const cJSON* capabilities) {
    auto vision = cJSON_GetObjectItem(capabilities, "vision");
    if (cJSON_IsObject(vision)) {
        auto url = cJSON_GetObjectItem(vision, "url");
        auto token = cJSON_GetObjectItem(vision, "token");
        if (cJSON_IsString(url)) {
            auto camera = Board::GetInstance().GetCamera();
            if (camera) {
                std::string url_str = std::string(url->valuestring);
                std::string token_str;
                if (cJSON_IsString(token)) {
                    token_str = std::string(token->valuestring);
                }
                camera->SetExplainUrl(url_str, token_str);
            }
        }
    }
}

void McpServer::ParseMessage(const cJSON* json) {
    // Check JSONRPC version
    auto version = cJSON_GetObjectItem(json, "jsonrpc");
    if (version == nullptr || !cJSON_IsString(version) || strcmp(version->valuestring, "2.0") != 0) {
        ESP_LOGE(TAG, "Invalid JSONRPC version: %s", version ? version->valuestring : "null");
        return;
    }
    
    // Check method
    auto method = cJSON_GetObjectItem(json, "method");
    if (method == nullptr || !cJSON_IsString(method)) {
        ESP_LOGE(TAG, "Missing method");
        return;
    }
    
    auto method_str = std::string(method->valuestring);
    if (method_str.find("notifications") == 0) {
        return;
    }
    
    // Check params
    auto params = cJSON_GetObjectItem(json, "params");
    if (params != nullptr && !cJSON_IsObject(params)) {
        ESP_LOGE(TAG, "Invalid params for method: %s", method_str.c_str());
        return;
    }

    auto id = cJSON_GetObjectItem(json, "id");
    if (id == nullptr || !cJSON_IsNumber(id)) {
        ESP_LOGE(TAG, "Invalid id for method: %s", method_str.c_str());
        return;
    }
    auto id_int = id->valueint;
    
    if (method_str == "initialize") {
        if (cJSON_IsObject(params)) {
            auto capabilities = cJSON_GetObjectItem(params, "capabilities");
            if (cJSON_IsObject(capabilities)) {
                ParseCapabilities(capabilities);
            }
        }
        auto app_desc = esp_app_get_description();
        std::string message = "{\"protocolVersion\":\"2024-11-05\",\"capabilities\":{\"tools\":{}},\"serverInfo\":{\"name\":\"" BOARD_NAME "\",\"version\":\"";
        message += app_desc->version;
        message += "\"}}";
        ReplyResult(id_int, message);
    } else if (method_str == "tools/list") {
        std::string cursor_str = "";
        if (params != nullptr) {
            auto cursor = cJSON_GetObjectItem(params, "cursor");
            if (cJSON_IsString(cursor)) {
                cursor_str = std::string(cursor->valuestring);
            }
        }
        GetToolsList(id_int, cursor_str);
    } else if (method_str == "tools/call") {
        if (!cJSON_IsObject(params)) {
            ESP_LOGE(TAG, "tools/call: Missing params");
            ReplyError(id_int, "Missing params");
            return;
        }
        auto tool_name = cJSON_GetObjectItem(params, "name");
        if (!cJSON_IsString(tool_name)) {
            ESP_LOGE(TAG, "tools/call: Missing name");
            ReplyError(id_int, "Missing name");
            return;
        }
        auto tool_arguments = cJSON_GetObjectItem(params, "arguments");
        if (tool_arguments != nullptr && !cJSON_IsObject(tool_arguments)) {
            ESP_LOGE(TAG, "tools/call: Invalid arguments");
            ReplyError(id_int, "Invalid arguments");
            return;
        }
        auto stack_size = cJSON_GetObjectItem(params, "stackSize");
        if (stack_size != nullptr && !cJSON_IsNumber(stack_size)) {
            ESP_LOGE(TAG, "tools/call: Invalid stackSize");
            ReplyError(id_int, "Invalid stackSize");
            return;
        }
        DoToolCall(id_int, std::string(tool_name->valuestring), tool_arguments, stack_size ? stack_size->valueint : DEFAULT_TOOLCALL_STACK_SIZE);
    } else {
        ESP_LOGE(TAG, "Method not implemented: %s", method_str.c_str());
        ReplyError(id_int, "Method not implemented: " + method_str);
    }
}

void McpServer::ReplyResult(int id, const std::string& result) {
    std::string payload = "{\"jsonrpc\":\"2.0\",\"id\":";
    payload += std::to_string(id) + ",\"result\":";
    payload += result;
    payload += "}";
    Application::GetInstance().SendMcpMessage(payload);
}

void McpServer::ReplyError(int id, const std::string& message) {
    std::string payload = "{\"jsonrpc\":\"2.0\",\"id\":";
    payload += std::to_string(id);
    payload += ",\"error\":{\"message\":\"";
    payload += message;
    payload += "\"}}";
    Application::GetInstance().SendMcpMessage(payload);
}

void McpServer::GetToolsList(int id, const std::string& cursor) {
    const int max_payload_size = 8000;
    std::string json = "{\"tools\":[";
    
    bool found_cursor = cursor.empty();
    auto it = tools_.begin();
    std::string next_cursor = "";
    
    while (it != tools_.end()) {
        // 如果我们还没有找到起始位置，继续搜索
        if (!found_cursor) {
            if ((*it)->name() == cursor) {
                found_cursor = true;
            } else {
                ++it;
                continue;
            }
        }
        
        // 添加tool前检查大小
        std::string tool_json = (*it)->to_json() + ",";
        if (json.length() + tool_json.length() + 30 > max_payload_size) {
            // 如果添加这个tool会超出大小限制，设置next_cursor并退出循环
            next_cursor = (*it)->name();
            break;
        }
        
        json += tool_json;
        ++it;
    }
    
    if (json.back() == ',') {
        json.pop_back();
    }
    
    if (json.back() == '[' && !tools_.empty()) {
        // 如果没有添加任何tool，返回错误
        ESP_LOGE(TAG, "tools/list: Failed to add tool %s because of payload size limit", next_cursor.c_str());
        ReplyError(id, "Failed to add tool " + next_cursor + " because of payload size limit");
        return;
    }

    if (next_cursor.empty()) {
        json += "]}";
    } else {
        json += "],\"nextCursor\":\"" + next_cursor + "\"}";
    }
    
    ReplyResult(id, json);
}

void McpServer::DoToolCall(int id, const std::string& tool_name, const cJSON* tool_arguments, int stack_size) {
    auto tool_iter = std::find_if(tools_.begin(), tools_.end(), 
                                 [&tool_name](const McpTool* tool) { 
                                     return tool->name() == tool_name; 
                                 });
    
    if (tool_iter == tools_.end()) {
        ESP_LOGE(TAG, "tools/call: Unknown tool: %s", tool_name.c_str());
        ReplyError(id, "Unknown tool: " + tool_name);
        return;
    }

    PropertyList arguments = (*tool_iter)->properties();
    try {
        for (auto& argument : arguments) {
            bool found = false;
            if (cJSON_IsObject(tool_arguments)) {
                auto value = cJSON_GetObjectItem(tool_arguments, argument.name().c_str());
                if (argument.type() == kPropertyTypeBoolean && cJSON_IsBool(value)) {
                    argument.set_value<bool>(value->valueint == 1);
                    found = true;
                } else if (argument.type() == kPropertyTypeInteger && cJSON_IsNumber(value)) {
                    argument.set_value<int>(value->valueint);
                    found = true;
                } else if (argument.type() == kPropertyTypeString && cJSON_IsString(value)) {
                    argument.set_value<std::string>(value->valuestring);
                    found = true;
                }
            }

            if (!argument.has_default_value() && !found) {
                ESP_LOGE(TAG, "tools/call: Missing valid argument: %s", argument.name().c_str());
                ReplyError(id, "Missing valid argument: " + argument.name());
                return;
            }
        }
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "tools/call: %s", e.what());
        ReplyError(id, e.what());
        return;
    }

    // Start a task to receive data with stack size
    esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
    cfg.thread_name = "tool_call";
    cfg.stack_size = stack_size;
    cfg.prio = 1;
    esp_pthread_set_cfg(&cfg);

    // Use a thread to call the tool to avoid blocking the main thread
    tool_call_thread_ = std::thread([this, id, tool_iter, arguments = std::move(arguments)]() {
        try {
            ReplyResult(id, (*tool_iter)->Call(arguments));
        } catch (const std::exception& e) {
            ESP_LOGE(TAG, "tools/call: %s", e.what());
            ReplyError(id, e.what());
        }
    });
    tool_call_thread_.detach();
}