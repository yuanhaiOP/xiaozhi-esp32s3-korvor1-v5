#!/usr/bin/env python3
"""
ESP32数据帧格式测试脚本
用于验证ESP32发送的数据帧格式和ROS2桥接节点的解析逻辑
"""

def create_move_frame(x_speed, y_speed):
    """创建移动指令数据帧"""
    # 将浮点数转换为整数（放大1000倍）
    x_int = int(x_speed * 1000)
    y_int = int(y_speed * 1000)
    
    # 构建数据帧
    frame = bytearray()
    frame.append(0xAA)                    # 帧头
    frame.append(0x01)                    # 移动命令
    frame.append((x_int >> 8) & 0xFF)    # X高字节
    frame.append(x_int & 0xFF)           # X低字节
    frame.append((y_int >> 8) & 0xFF)    # Y高字节
    frame.append(y_int & 0xFF)           # Y低字节
    
    # 计算校验和（不包括帧头）
    checksum = 0
    for i in range(1, len(frame)):
        checksum ^= frame[i]
    frame.append(checksum)                # 校验和
    
    frame.append(0x55)                    # 帧尾
    
    return frame

def create_dance_frame():
    """创建跳舞指令数据帧"""
    frame = bytearray()
    frame.append(0xAA)                    # 帧头
    frame.append(0x02)                    # 跳舞命令
    frame.append(0x00)                    # 数据高字节（未使用）
    frame.append(0x00)                    # 数据低字节（未使用）
    frame.append(0x00)                    # 数据高字节（未使用）
    frame.append(0x00)                    # 数据低字节（未使用）
    
    # 计算校验和
    checksum = 0
    for i in range(1, len(frame)):
        checksum ^= frame[i]
    frame.append(checksum)                # 校验和
    
    frame.append(0x55)                    # 帧尾
    
    return frame

def create_light_frame(mode):
    """创建灯光指令数据帧"""
    frame = bytearray()
    frame.append(0xAA)                    # 帧头
    frame.append(0x03)                    # 灯光命令
    frame.append(0x00)                    # 数据高字节（未使用）
    frame.append(0x00)                    # 数据低字节（未使用）
    frame.append(0x00)                    # 数据高字节（未使用）
    frame.append(mode)                    # 灯光模式
    
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
    frame.append(0x00)                    # 数据高字节（未使用）
    frame.append(0x00)                    # 数据低字节（未使用）
    frame.append(0x00)                    # 数据高字节（未使用）
    frame.append(0x00)                    # 数据低字节（未使用）
    
    # 计算校验和
    checksum = 0
    for i in range(1, len(frame)):
        checksum ^= frame[i]
    frame.append(checksum)                # 校验和
    
    frame.append(0x55)                    # 帧尾
    
    return frame

def print_frame(frame, description):
    """打印数据帧"""
    print(f"\n{description}:")
    print(" ".join([f"{b:02X}" for b in frame]))
    print(f"Length: {len(frame)} bytes")
    
    # 解析帧内容
    if len(frame) >= 8:
        header = frame[0]
        cmd = frame[1]
        x_raw = (frame[2] << 8) | frame[3]
        y_raw = (frame[4] << 8) | frame[5]
        checksum = frame[6]
        tail = frame[7]
        
        print(f"Header: 0x{header:02X}")
        print(f"Command: 0x{cmd:02X}")
        print(f"X raw: {x_raw} (speed: {x_raw/1000.0:.3f} m/s)")
        print(f"Y raw: {y_raw} (speed: {y_raw/1000.0:.3f} m/s)")
        print(f"Checksum: 0x{checksum:02X}")
        print(f"Tail: 0x{tail:02X}")

def main():
    """主函数"""
    print("ESP32数据帧格式测试")
    print("=" * 50)
    
    # 测试移动指令
    move_frame = create_move_frame(0.0, 0.2)  # 前进0.2m/s
    print_frame(move_frame, "前进指令")
    
    # 测试左转指令
    turn_left_frame = create_move_frame(-0.5, 0.0)  # 左转0.5rad/s
    print_frame(turn_left_frame, "左转指令")
    
    # 测试跳舞指令
    dance_frame = create_dance_frame()
    print_frame(dance_frame, "跳舞指令")
    
    # 测试灯光指令
    light_frame = create_light_frame(3)  # 3号灯光模式
    print_frame(light_frame, "灯光指令")
    
    # 测试停止指令
    stop_frame = create_stop_frame()
    print_frame(stop_frame, "停止指令")
    
    print("\n" + "=" * 50)
    print("测试完成！")
    print("这些数据帧可以直接发送给ROS2桥接节点进行测试。")

if __name__ == "__main__":
    main() 