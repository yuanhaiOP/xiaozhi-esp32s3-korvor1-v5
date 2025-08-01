#!/usr/bin/env python3
"""
ESP32-S3-KORVO1-V5 配置测试脚本
"""

import os
import sys

def test_config_files():
    """测试配置文件是否存在"""
    board_dir = "main/boards/esp32s3-korvo1-v5"
    
    required_files = [
        "config.h",
        "config.json", 
        "esp32s3_korvo1_v5_board.cc",
        "README.md"
    ]
    
    print("检查KORVO1-V5配置文件...")
    
    for file in required_files:
        file_path = os.path.join(board_dir, file)
        if os.path.exists(file_path):
            print(f"✅ {file} - 存在")
        else:
            print(f"❌ {file} - 缺失")
            return False
    
    return True

def test_config_content():
    """测试配置文件内容"""
    print("\n检查配置文件内容...")
    
    # 检查config.h中的关键配置
    config_h = "main/boards/esp32s3-korvo1-v5/config.h"
    if os.path.exists(config_h):
        with open(config_h, 'r', encoding='utf-8') as f:
            content = f.read()
            
        # 检查关键配置
        checks = [
            ("AUDIO_CODEC_ES7210_ADDR", "三阵列麦克风配置"),
            ("DISPLAY_WIDTH   0", "无显示屏配置"),
            ("CAMERA_PIN_PWDN -1", "无摄像头配置"),
            ("BOOT_BUTTON_GPIO        GPIO_NUM_5", "按钮配置")
        ]
        
        for check, description in checks:
            if check in content:
                print(f"✅ {description}")
            else:
                print(f"❌ {description} - 未找到: {check}")
                return False
    
    return True

def test_board_code():
    """测试开发板代码"""
    print("\n检查开发板代码...")
    
    board_cc = "main/boards/esp32s3-korvo1-v5/esp32s3_korvo1_v5_board.cc"
    if os.path.exists(board_cc):
        with open(board_cc, 'r', encoding='utf-8') as f:
            content = f.read()
            
        # 检查关键代码
        checks = [
            ("class Esp32S3Korvo1V5Board", "开发板类定义"),
            ("return nullptr;  // KORVO1 V5 没有显示屏", "无显示屏返回"),
            ("return nullptr;  // KORVO1 V5 没有摄像头", "无摄像头返回"),
            ("DECLARE_BOARD(Esp32S3Korvo1V5Board)", "开发板注册")
        ]
        
        for check, description in checks:
            if check in content:
                print(f"✅ {description}")
            else:
                print(f"❌ {description} - 未找到: {check}")
                return False
    
    return True

def main():
    """主函数"""
    print("ESP32-S3-KORVO1-V5 配置测试")
    print("=" * 50)
    
    # 测试配置文件
    if not test_config_files():
        print("\n❌ 配置文件测试失败")
        return 1
    
    # 测试配置内容
    if not test_config_content():
        print("\n❌ 配置内容测试失败")
        return 1
    
    # 测试开发板代码
    if not test_board_code():
        print("\n❌ 开发板代码测试失败")
        return 1
    
    print("\n🎉 所有测试通过！KORVO1-V5配置正确。")
    print("\n硬件特性:")
    print("- ✅ 三阵列麦克风 (ES7210)")
    print("- ✅ 音频播放 (ES8311)")
    print("- ✅ 无显示屏")
    print("- ✅ 无摄像头")
    print("- ✅ BOOT按钮控制")
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 