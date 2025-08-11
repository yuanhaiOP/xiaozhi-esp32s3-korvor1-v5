# ESP32 机器人桥接 Launch 文件使用指南

## 概述

Launch 文件是 ROS2 中用于启动多个节点和配置参数的标准方式。本指南将详细介绍如何使用 `esp32_bridge.launch.py` 文件来启动 ESP32 机器人桥接节点。

## 文件结构

```
ros_esp32_bridge/
├── launch/
│   └── esp32_bridge.launch.py    # 主要的 launch 文件
├── config/
│   └── bridge_config.yaml         # 配置文件
└── src/
    └── esp32_robot_bridge.cpp    # 节点源代码
```

## 基本使用方法

### 1. 使用默认参数启动

```bash
# 使用默认串口 /dev/ttyUSB0
ros2 launch esp32_robot_bridge esp32_bridge.launch.py

# 指定不同的串口设备
ros2 launch esp32_robot_bridge esp32_bridge.launch.py serial_port:=/dev/ttyACM0
```

### 2. 自定义参数启动

```bash
# 指定串口和波特率
ros2 launch esp32_robot_bridge esp32_bridge.launch.py \
    serial_port:=/dev/ttyUSB0 \
    baud_rate:=115200

# 调整控制参数
ros2 launch esp32_robot_bridge esp32_bridge.launch.py \
    serial_port:=/dev/ttyUSB0 \
    position_tolerance:=0.05 \
    angle_tolerance:=0.05 \
    linear_p_gain:=3.0 \
    angular_p_gain:=4.0
```

## 参数详解

### 串口参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `serial_port` | `/dev/ttyUSB0` | ESP32 连接的串口设备 |
| `baud_rate` | `115200` | 串口波特率 |

### 位置控制参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `position_tolerance` | `0.1` | 位置容差（米），到达目标位置的判定阈值 |
| `angle_tolerance` | `0.1` | 角度容差（弧度），到达目标角度的判定阈值 |
| `linear_p_gain` | `2.0` | 线速度 P 增益，控制位置到速度的转换 |
| `angular_p_gain` | `3.0` | 角速度 P 增益，控制角度到角速度的转换 |

### 加速度限制参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `max_linear_accel` | `1.0` | 最大线加速度（m/s²） |
| `max_angular_accel` | `2.0` | 最大角加速度（rad/s²） |

## 高级使用方法

### 1. 使用配置文件

创建自定义配置文件 `my_config.yaml`：

```yaml
esp32_robot_bridge:
  ros__parameters:
    serial_port: "/dev/ttyUSB0"
    baud_rate: 115200
    position_tolerance: 0.05
    angle_tolerance: 0.05
    linear_p_gain: 3.0
    angular_p_gain: 4.0
    max_linear_accel: 0.5
    max_angular_accel: 1.0
```

然后使用配置文件启动：

```bash
ros2 launch esp32_robot_bridge esp32_bridge.launch.py \
    --load-config my_config.yaml
```

### 2. 调试模式启动

```bash
# 显示详细日志
ros2 launch esp32_robot_bridge esp32_bridge.launch.py \
    --ros-args --log-level debug

# 或者直接运行节点进行调试
ros2 run esp32_robot_bridge esp32_robot_bridge \
    --ros-args \
    -p serial_port:=/dev/ttyUSB0 \
    -p position_tolerance:=0.05 \
    --log-level debug
```

### 3. 多设备启动

如果有多个 ESP32 设备，可以启动多个节点：

```bash
# 终端1：启动第一个设备
ros2 launch esp32_robot_bridge esp32_bridge.launch.py \
    serial_port:=/dev/ttyUSB0 \
    --ros-args -r __ns:=/robot1

# 终端2：启动第二个设备
ros2 launch esp32_robot_bridge esp32_bridge.launch.py \
    serial_port:=/dev/ttyUSB1 \
    --ros-args -r __ns:=/robot2
```

## 监控和调试

### 1. 查看节点状态

```bash
# 列出所有节点
ros2 node list

# 查看节点信息
ros2 node info /esp32_robot_bridge

# 查看节点参数
ros2 param list /esp32_robot_bridge
```

### 2. 监控话题

```bash
# 查看所有话题
ros2 topic list

# 监控 Twist 消息
ros2 topic echo /cmd_vel

# 查看话题信息
ros2 topic info /cmd_vel
```

### 3. 查看日志

```bash
# 查看节点日志
ros2 run esp32_robot_bridge esp32_robot_bridge --ros-args --log-level debug

# 或者使用 launch 文件时添加日志级别
ros2 launch esp32_robot_bridge esp32_bridge.launch.py \
    --ros-args --log-level debug
```

## 常见问题解决

### 1. 串口权限问题

```bash
# 添加用户到 dialout 组
sudo usermod -a -G dialout $USER

# 临时修改权限
sudo chmod 666 /dev/ttyUSB0

# 重新登录使权限生效
logout
# 重新登录
```

### 2. 串口设备不存在

```bash
# 检查 USB 设备
lsusb

# 检查串口设备
ls -l /dev/ttyUSB*
ls -l /dev/ttyACM*

# 查看内核消息
dmesg | grep tty
```

### 3. 节点启动失败

```bash
# 检查包是否正确安装
ros2 pkg list | grep esp32

# 检查可执行文件
ros2 run esp32_robot_bridge esp32_robot_bridge --help

# 查看详细错误信息
ros2 launch esp32_robot_bridge esp32_bridge.launch.py --debug
```

## 测试示例

### 1. 基本功能测试

```bash
# 启动桥接节点
ros2 launch esp32_robot_bridge esp32_bridge.launch.py

# 在另一个终端监控消息
ros2 topic echo /cmd_vel

# 在 ESP32 上发送语音命令："前进5米"
# 应该看到 Twist 消息输出
```

### 2. 参数调整测试

```bash
# 使用更严格的容差
ros2 launch esp32_robot_bridge esp32_bridge.launch.py \
    position_tolerance:=0.02 \
    angle_tolerance:=0.02

# 使用更高的增益
ros2 launch esp32_robot_bridge esp32_bridge.launch.py \
    linear_p_gain:=5.0 \
    angular_p_gain:=6.0
```

## 最佳实践

1. **参数调优**：根据实际机器人性能调整 P 增益和容差参数
2. **日志监控**：使用适当的日志级别进行调试
3. **错误处理**：检查串口连接和权限设置
4. **性能优化**：根据机器人响应特性调整加速度限制
5. **安全考虑**：设置合理的速度限制，避免机器人失控

## 总结

Launch 文件提供了灵活且可重复的节点启动方式。通过合理配置参数，可以轻松适应不同的硬件环境和控制需求。 