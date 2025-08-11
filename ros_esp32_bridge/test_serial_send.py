#!/usr/bin/env python3
"""
串口发送测试脚本
用于模拟ESP32发送数据帧到ROS2桥接节点
"""

import serial
import time
import sys

def create_move_frame(x_speed, y_speed):
    """创建移动指令数据帧"""
    x_int = int(x_speed * 1000)
    y_int = int(y_speed * 1000)
    
    frame = bytearray()
    frame.append(0xAA)                    # 帧头
    frame.append(0x01)                    # 移动命令
    frame.append((x_int >> 8) & 0xFF)    # X高字节
    frame.append(x_int & 0xFF)           # X低字节
    frame.append((y_int >> 8) & 0xFF)    # Y高字节
    frame.append(y_int & 0xFF)           # Y低字节
    
    # 计算校验和
    checksum = 0
    for i in range(1, len(frame)):
        checksum ^= frame[i]
    frame.append(checksum)                # 校验和
    
    frame.append(0x55)                    # 帧尾
    return frame

def create_stop_frame():
    """创建停止指令数据帧"""
    frame = bytearray()
    frame.append(0xAA)                    # 帧头
    frame.append(0x04)                    # 停止命令
    frame.append(0x00)                    # 数据高字节
    frame.append(0x00)                    # 数据低字节
    frame.append(0x00)                    # 数据高字节
    frame.append(0x00)                    # 数据低字节
    
    # 计算校验和
    checksum = 0
    for i in range(1, len(frame)):
        checksum ^= frame[i]
    frame.append(checksum)                # 校验和
    
    frame.append(0x55)                    # 帧尾
    return frame

def send_frame(ser, frame, description):
    """发送数据帧"""
    print(f"发送 {description}: {' '.join([f'{b:02X}' for b in frame])}")
    ser.write(frame)
    time.sleep(0.1)  # 等待发送完成

def main():
    # 串口配置
    port = "/dev/ttyUSB0"
    baud_rate = 115200
    
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    print(f"连接到串口: {port}")
    print(f"波特率: {baud_rate}")
    
    try:
        # 打开串口
        ser = serial.Serial(port, baud_rate, timeout=1)
        print("✓ 串口连接成功")
        
        # 等待串口稳定
        time.sleep(2)
        
        print("\n=== 开始发送测试数据帧 ===")
        
        # 测试1: 前进
        print("\n1. 发送前进指令 (0.2 m/s)")
        forward_frame = create_move_frame(0.0, 0.2)
        send_frame(ser, forward_frame, "前进指令")
        time.sleep(3)
        
        # 测试2: 左转
        print("\n2. 发送左转指令 (-0.5 rad/s)")
        turn_left_frame = create_move_frame(-0.5, 0.0)
        send_frame(ser, turn_left_frame, "左转指令")
        time.sleep(3)
        
        # 测试3: 停止
        print("\n3. 发送停止指令")
        stop_frame = create_stop_frame()
        send_frame(ser, stop_frame, "停止指令")
        time.sleep(1)
        
        # 测试4: 后退
        print("\n4. 发送后退指令 (-0.3 m/s)")
        backward_frame = create_move_frame(0.0, -0.3)
        send_frame(ser, backward_frame, "后退指令")
        time.sleep(3)
        
        # 测试5: 最终停止
        print("\n5. 发送最终停止指令")
        final_stop_frame = create_stop_frame()
        send_frame(ser, final_stop_frame, "最终停止指令")
        
        print("\n✓ 测试完成！")
        print("请检查ROS2话题 /cmd_vel 和 /esp32_debug 的输出")
        
    except serial.SerialException as e:
        print(f"错误: 无法打开串口 {port}")
        print(f"详细信息: {e}")
        print("\n可能的解决方案:")
        print("1. 检查串口设备是否存在: ls /dev/ttyUSB*")
        print("2. 检查串口权限: sudo chmod 666 /dev/ttyUSB0")
        print("3. 检查用户是否在dialout组: groups $USER")
        print("4. 检查串口是否被占用: sudo fuser -k /dev/ttyUSB0")
        
    except KeyboardInterrupt:
        print("\n用户中断测试")
        
    finally:
        if 'ser' in locals():
            ser.close()
            print("串口已关闭")

if __name__ == "__main__":
    main() 