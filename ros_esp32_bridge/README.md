# ESP32 Robot Bridge (ROS2)

这是一个ROS2节点，用于接收ESP32发送的机器人控制指令，并将其转换为ROS2的Twist消息。

## 功能特性

- 通过串口接收ESP32发送的数据帧
- 解析标准数据帧格式（帧头+命令+数据+校验和+帧尾）
- 支持移动、跳舞、灯光、停止等命令
- **将目标位置转换为实时速度控制**
- 将解析后的速度信息发布为ROS2 Twist消息
- 提供调试信息发布

## 核心转换逻辑

### 问题分析
ESP32发送的是**目标位置指令**（如"前进5米"、"左转90度"），而ROS需要的是**实时速度**（Twist消息）。

### 解决方案
实现了**目标位置到实时速度的转换**：

1. **速度指令解析**：ESP32发送速度指令（如0.2m/s前进）
2. **目标位置计算**：根据速度指令计算目标位置（如以0.2m/s速度移动5秒到达1米位置）
3. **位置控制器**：使用P控制器计算实时速度
4. **速度发布**：将计算出的实时速度发布为Twist消息

### 转换流程
```
ESP32速度指令 → 目标位置计算 → P控制器 → 实时速度 → Twist消息
     ↓              ↓              ↓          ↓         ↓
  0.2m/s前进    →  目标1米    →  误差控制  →  实时速度  →  cmd_vel
```

## ESP32数据帧格式适配

根据ESP32的`robot_movement_controller.cc`代码，数据帧格式为：

```
字节位置  含义          示例值      说明
0        帧头1         0xAA        固定帧头
1        命令类型      0x01        0x01=移动, 0x02=跳舞, 0x03=灯光, 0x04=停止
2-3      X值(高-低)    0x0000      X角速度(放大1000倍)
4-5      Y值(高-低)    0x00C8      Y线速度(放大1000倍)
6        校验和        0xXX        异或校验（不包括帧头）
7        帧尾          0x55        固定帧尾
```

**注意**：ESP32发送的格式与之前不同，帧头只有0xAA，没有0x55，命令类型在第二个字节。

## 控制参数

### 位置控制参数
| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| position_tolerance | 0.05 | 位置容差 (米) |
| angle_tolerance | 0.1 | 角度容差 (弧度) |
| linear_p_gain | 2.0 | 线速度P增益 |
| angular_p_gain | 3.0 | 角速度P增益 |

### 速度限制参数
| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| max_linear_speed | 1.0 | 最大线速度 (m/s) |
| max_angular_speed | 2.0 | 最大角速度 (rad/s) |
| max_linear_accel | 0.5 | 最大线加速度 (m/s²) |
| max_angular_accel | 1.0 | 最大角加速度 (rad/s²) |

## 安装依赖

```bash
# 安装Boost库
sudo apt-get install libboost-all-dev

# 安装ROS2依赖
sudo apt-get install ros-humble-geometry-msgs ros-humble-std-msgs
```

## 编译

```bash
# 进入ROS2工作空间
cd ~/ros2_ws/src

# 克隆或复制此包
cp -r ros_esp32_bridge ./

# 编译
cd ~/ros2_ws
colcon build --packages-select esp32_robot_bridge
```

## 使用方法

### 1. 启动节点

```bash
# 使用默认配置启动
ros2 launch esp32_robot_bridge esp32_bridge.launch.py

# 或者指定参数
ros2 launch esp32_robot_bridge esp32_bridge.launch.py port_name:=/dev/ttyUSB0 baud_rate:=115200
```

### 2. 查看发布的话题

```bash
# 查看cmd_vel话题
ros2 topic echo /cmd_vel

# 查看调试信息
ros2 topic echo /esp32_debug

# 查看话题列表
ros2 topic list
```

### 3. 测试机器人控制

```bash
# 手动发布速度命令测试
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 配置参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| port_name | /dev/ttyUSB0 | 串口设备名 |
| baud_rate | 115200 | 波特率 |
| max_linear_speed | 1.0 | 最大线速度 (m/s) |
| max_angular_speed | 2.0 | 最大角速度 (rad/s) |
| linear_scale_factor | 1.0 | 线速度缩放因子 |
| angular_scale_factor | 1.0 | 角速度缩放因子 |
| position_tolerance | 0.05 | 位置容差 (米) |
| angle_tolerance | 0.1 | 角度容差 (弧度) |
| linear_p_gain | 2.0 | 线速度P增益 |
| angular_p_gain | 3.0 | 角速度P增益 |
| max_linear_accel | 0.5 | 最大线加速度 (m/s²) |
| max_angular_accel | 1.0 | 最大角加速度 (rad/s²) |

## 支持的命令

### 移动命令 (CMD_MOVE = 0x01)
- 解析X和Y速度值
- 转换为目标位置
- 使用P控制器计算实时速度
- 发布Twist消息

### 跳舞命令 (CMD_DANCE = 0x02)
- 设置目标角度（旋转180度）
- 使用角速度P控制器
- 线速度设为0

### 灯光命令 (CMD_LIGHT = 0x03)
- 解析灯光模式
- 仅记录日志，不改变运动状态

### 停止命令 (CMD_STOP = 0x04)
- 立即停止所有运动
- 所有速度设为0

## 控制算法

### P控制器
```cpp
// 位置误差
double dx = target_x - current_x;
double dy = target_y - current_y;
double dtheta = target_theta - current_theta;

// 目标速度
double target_linear_x = linear_p_gain * dx;
double target_linear_y = linear_p_gain * dy;
double target_angular_z = angular_p_gain * dtheta;
```

### 速度限制
```cpp
// 限制最大线速度
double linear_speed = sqrt(target_linear_x² + target_linear_y²);
if (linear_speed > max_linear_speed) {
    double scale = max_linear_speed / linear_speed;
    target_linear_x *= scale;
    target_linear_y *= scale;
}

// 限制最大角速度
if (|target_angular_z| > max_angular_speed) {
    target_angular_z = sign(target_angular_z) * max_angular_speed;
}
```

### 到达判断
```cpp
// 位置误差
double position_error = sqrt(dx² + dy²);
double angle_error = |dtheta|;

// 到达目标
bool reached = (position_error < position_tolerance) && 
               (angle_error < angle_tolerance);
```

## 故障排除

### 串口权限问题
```bash
# 添加用户到dialout组
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

### 串口设备不存在
```bash
# 查看可用串口
ls /dev/ttyUSB*
ls /dev/ttyACM*

# 修改launch文件中的port_name参数
```

### 数据帧解析错误
- 检查ESP32发送的数据帧格式
- 确认波特率设置正确
- 查看调试信息确认数据接收

### 控制效果不佳
- 调整P增益参数：`linear_p_gain`、`angular_p_gain`
- 调整容差参数：`position_tolerance`、`angle_tolerance`
- 调整加速度限制：`max_linear_accel`、`max_angular_accel`

## 与robot_driver.cpp的集成

此桥接节点可以与现有的robot_driver.cpp配合使用：

1. ESP32发送控制指令 → ESP32 Bridge接收
2. ESP32 Bridge发布cmd_vel → robot_driver订阅
3. robot_driver控制机器人底盘

形成完整的控制链路：ESP32 → Bridge → ROS2 → robot_driver → 机器人底盘

## 数据帧解析说明

ESP32发送的数据帧格式：
```
AA 01 00 00 00 C8 XX 55
│  │  │     │     │  │
│  │  │     │     │  └─ 帧尾
│  │  │     │     └──── 校验和
│  │  │     └────────── Y线速度(0x00C8 = 200 = 0.2m/s)
│  │  └──────────────── X角速度(0x0000 = 0 = 0rad/s)
│  └──────────────────── 移动命令
└──────────────────────── 帧头
```

ROS2桥接节点会：
1. 解析这个数据帧
2. 将速度指令转换为目标位置
3. 使用P控制器计算实时速度
4. 发布相应的Twist消息 