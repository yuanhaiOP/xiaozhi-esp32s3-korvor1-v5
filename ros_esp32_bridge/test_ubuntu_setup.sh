#!/bin/bash

echo "=== ESP32 Robot Bridge Ubuntu 测试脚本 ==="

# 检查ROS2环境
if ! command -v ros2 &> /dev/null; then
    echo "错误: ROS2未安装，请先安装ROS2 Humble"
    exit 1
fi

echo "✓ ROS2环境检查通过"

# 检查串口设备
echo "检查串口设备..."
ls /dev/ttyUSB* 2>/dev/null || echo "警告: 未找到 /dev/ttyUSB* 设备"
ls /dev/ttyACM* 2>/dev/null || echo "警告: 未找到 /dev/ttyACM* 设备"

# 检查用户权限
if ! groups $USER | grep -q dialout; then
    echo "警告: 用户不在dialout组中，可能无法访问串口"
    echo "请运行: sudo usermod -a -G dialout $USER"
fi

echo ""
echo "=== 测试步骤 ==="
echo "1. 编译ROS2包:"
echo "   cd ~/ros2_ws"
echo "   colcon build --packages-select esp32_robot_bridge"
echo "   source install/setup.bash"
echo ""
echo "2. 测试数据帧格式:"
echo "   python3 test_frame_format.py"
echo ""
echo "3. 启动桥接节点:"
echo "   ros2 launch esp32_robot_bridge esp32_bridge.launch.py"
echo ""
echo "4. 监控话题 (新终端):"
echo "   source ~/ros2_ws/install/setup.bash"
echo "   ros2 topic echo /cmd_vel"
echo "   ros2 topic echo /esp32_debug"
echo ""
echo "5. 模拟ESP32发送数据:"
echo "   python3 test_serial_send.py"
echo ""
echo "=== 故障排除 ==="
echo "• 串口权限问题: sudo chmod 666 /dev/ttyUSB0"
echo "• 串口被占用: sudo fuser -k /dev/ttyUSB0"
echo "• 查看串口数据: sudo cat /dev/ttyUSB0"
echo "• 测试串口: minicom -D /dev/ttyUSB0" 