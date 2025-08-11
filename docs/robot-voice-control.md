# 机器人语音控制功能说明

## 概述

本功能允许用户通过语音指令控制机器人的移动，系统会自动识别用户的语音指令，转换为精确的机器人控制命令，并通过UART0发送给机器人底盘。

## 功能特性

### 1. 智能语音指令解析
- 支持中英文混合指令
- 自动识别移动方向、速度、持续时间等参数
- 支持自然语言表达

### 2. 支持的指令类型

#### 基础移动指令
- **前进**: "前进", "向前", "往前走", "向前走", "直走", "go forward", "forward", "go ahead"
- **后退**: "后退", "向后", "往后走", "向后走", "go back", "backward", "back"
- **左转**: "左转", "向左转", "向左", "turn left", "left", "go left"
- **右转**: "右转", "向右转", "向右", "turn right", "right", "go right"
- **停止**: "停止", "停下", "停", "stop", "halt"

#### 速度控制
- **慢速**: "慢", "slow" (速度: 0.3)
- **中速**: "中速", "medium" (速度: 0.6)
- **快速**: "快", "fast" (速度: 1.0)

#### 时间控制
- **持续时间**: "3秒", "5 seconds", "10s" 等

#### 灯光控制
- **开灯**: "开灯", "打开灯", "亮灯", "turn on light", "light on"
- **关灯**: "关灯", "关闭灯", "灭灯", "turn off light", "light off"
- **特定模式**: "3号灯", "5号灯" 等 (模式1-6)

#### 特殊功能
- **跳舞**: "跳舞", "dance", "show dance"

## 使用示例

### 基础移动指令
```
用户: "前进"
机器人: 以默认速度向前移动

用户: "快一点左转"
机器人: 快速向左转

用户: "慢速前进3秒"
机器人: 慢速前进3秒后停止
```

### 复杂指令
```
用户: "打开3号灯"
机器人: 开启灯光模式3

用户: "跳舞"
机器人: 执行跳舞动作

用户: "停止"
机器人: 立即停止所有动作

用户: "快一点左转"
机器人: 发送 "x-1.000 y0.000" (快速左转)

用户: "慢速前进"
机器人: 发送 "x0.000 y0.100" (慢速前进)
```

### MCP工具调用示例

#### 1. 语音控制
```json
{
  "jsonrpc": "2.0",
  "method": "tools/call",
  "params": {
    "name": "self.robot.control_by_voice",
    "arguments": {
      "voice_command": "快一点左转"
    }
  },
  "id": 1
}
```

#### 2. 直接速度控制
```json
{
  "jsonrpc": "2.0",
  "method": "tools/call",
  "params": {
    "name": "self.robot.set_velocity",
    "arguments": {
      "linear_velocity": 30,
      "angular_velocity": -50
    }
  },
  "id": 2
}
```

## MCP工具接口

### 1. 语音控制工具
```json
{
  "name": "self.robot.control_by_voice",
  "description": "Control robot movement through voice commands",
  "arguments": {
    "voice_command": "string"
  }
}
```

### 2. 直接移动控制
```json
{
  "name": "self.robot.move",
  "description": "Direct robot movement control with linear and angular velocity",
  "arguments": {
    "direction": "string (forward/backward/left/right/stop)",
    "linear_speed": "integer (0-100, represents 0.0-1.0 m/s)",
    "angular_speed": "integer (0-200, represents 0.0-2.0 rad/s)",
    "duration": "integer (0-60 seconds)"
  }
}
```

### 3. 直接速度控制
```json
{
  "name": "self.robot.set_velocity",
  "description": "Set robot linear and angular velocity directly",
  "arguments": {
    "linear_velocity": "integer (-100 to 100, represents -1.0 to 1.0 m/s)",
    "angular_velocity": "integer (-200 to 200, represents -2.0 to 2.0 rad/s)"
  }
}
```

### 3. 灯光控制
```json
{
  "name": "self.robot.light",
  "description": "Control robot light effects",
  "arguments": {
    "mode": "integer (0-6)"
  }
}
```

### 4. 灯光控制
```json
{
  "name": "self.robot.light",
  "description": "Control robot light effects",
  "arguments": {
    "mode": "integer (0-6)"
  }
}
```

### 5. 停止控制
```json
{
  "name": "self.robot.stop",
  "description": "Stop all robot movement",
  "arguments": {}
}
```

## 硬件连接

### UART配置
- **端口**: UART0（使用系统已配置的UART0）
- **TX引脚**: 系统默认TXD0引脚
- **RX引脚**: 系统默认RXD0引脚
- **波特率**: 115200
- **数据位**: 8
- **停止位**: 1
- **校验位**: 无

### 机器人底盘协议
系统发送标准数据帧格式的命令给机器人底盘：

#### 数据帧格式
```
字节位置  含义          示例值      说明
0        帧头          0xAA        固定帧头
1        命令类型      0x01        0x01=移动, 0x02=跳舞, 0x03=灯光, 0x04=停止
2-3      X值(高-低)    0xFC18      X角速度(放大1000倍)
4-5      Y值(高-低)    0x0000      Y线速度(放大1000倍)
6        校验和        0xXX        异或校验
7        帧尾          0x55        固定帧尾
```

#### 命令示例
```
移动命令 (8字节数据帧):
- 前进: AA 01 00 00 00 C8 01 C9 55 (x=0.000, y=0.200)
- 后退: AA 01 00 00 00 38 FE 39 55 (x=0.000, y=-0.200)
- 左转: AA 01 F4 01 00 00 01 F6 55 (x=-0.500, y=0.000)
- 右转: AA 01 0C FE 00 00 FE 0A 55 (x=0.500, y=0.000)
- 停止: AA 01 00 00 00 00 01 01 55 (x=0.000, y=0.000)

特殊命令:
- 跳舞: AA 02 00 00 00 00 02 02 55
- 灯光: AA 03 00 00 00 03 03 03 55 (模式3)
- 停止: AA 04 00 00 00 00 04 04 55
```

#### 数据转换说明
- 浮点数放大1000倍转换为16位整数
- 例如: -1.000 → -1000 → 0xFC18 (高字节0xFC, 低字节0x18)
- 校验和: 对命令类型+数据字节进行异或运算

## 配置说明

### 修改UART引脚
在 `main/robot_config.h` 中修改以下定义：
```c
// 使用系统已配置的UART0（推荐）
#define ROBOT_UART_PORT        UART_NUM_0
#define ROBOT_UART_TX_PIN      UART_PIN_NO_CHANGE  // 使用系统默认引脚
#define ROBOT_UART_RX_PIN      UART_PIN_NO_CHANGE  // 使用系统默认引脚

// 或使用其他UART端口
// #define ROBOT_UART_PORT        UART_NUM_1
// #define ROBOT_UART_TX_PIN      43  // 修改为实际使用的TX引脚
// #define ROBOT_UART_RX_PIN      44  // 修改为实际使用的RX引脚
```

### 添加新的语音指令
在 `main/robot_movement_controller.cc` 的 `InitializeCommandMap()` 方法中添加新的指令映射：
```cpp
command_map_["新指令"] = MovementCommand::FORWARD;
```

## 调试信息

系统会输出详细的调试信息：
```
I (1234) RobotController: Parsing voice command: '前进'
I (1234) RobotController: Matched command: 前进
I (1234) RobotController: Sending UART command: x0.0 y1.0
I (1234) RobotController: UART command sent successfully, length: 8
```

## 注意事项

1. **语音识别**: 确保语音识别准确，避免误识别
2. **安全控制**: 建议在开阔空间使用，避免碰撞
3. **电池管理**: 注意机器人电池电量，及时充电
4. **网络连接**: 确保设备网络连接正常，以便接收语音指令

## 故障排除

### 常见问题
1. **机器人不响应**: 检查UART连接和机器人电源
2. **指令识别错误**: 检查语音识别质量和指令映射
3. **移动异常**: 检查机器人底盘状态和传感器

### 调试步骤
1. 查看串口日志确认指令解析
2. 检查UART通信状态
3. 验证机器人底盘响应
4. 测试语音识别准确性 