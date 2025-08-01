# ESP32-S3-KORVO1-V5 开发板

## 概述

ESP32-S3-KORVO1-V5 是乐鑫官方推出的语音开发板，专为语音交互应用设计。本配置将小智AI语音聊天机器人移植到该开发板上。

**参考文档**: [乐鑫官方KORVO1用户指南](https://github.com/espressif/esp-skainet/blob/master/docs/zh_CN/hw-reference/esp32s3/user-guide-korvo-1.md)

## 硬件特性

### 主要组件
- **主控芯片**: ESP32-S3
- **音频编解码器**: ES8311 (播放) + ES7210 (三阵列麦克风)
- **麦克风**: 三阵列麦克风，支持声源定位和降噪
- **按钮**: BOOT按钮 (GPIO5)
- **无显示屏**: 纯语音交互设计
- **无摄像头**: 专注于音频处理
- **I2C**: 用于音频编解码器通信
- **I2S**: 用于音频数据传输

### 引脚配置

#### 音频I2S引脚
- MCLK: GPIO16
- WS: GPIO45  
- BCLK: GPIO9
- DIN: GPIO10
- DOUT: GPIO8

#### I2C引脚 (音频编解码器)
- SDA: GPIO17
- SCL: GPIO18

#### 三阵列麦克风配置
- ES7210支持多通道音频输入
- 支持声源定位和降噪算法
- 支持远场语音识别

## 编译配置

### 音频配置
KORVO1-V5专注于音频处理，支持三阵列麦克风：

```bash
idf.py menuconfig
# 进入 Component config -> Audio -> 配置音频参数
```

### 编译命令
```bash
# 编译固件
python scripts/release.py esp32s3-korvo1-v5

# 或者直接使用idf.py
idf.py set-target esp32s3
idf.py build
```

## 烧录固件

### 使用esptool烧录
```bash
esptool.py --chip esp32s3 --port COM端口 --baud 921600 \
    --before default_reset --after hard_reset \
    write_flash -z --flash_mode dio --flash_freq 80m --flash_size 8MB \
    0x0 bootloader.bin \
    0x8000 partition-table.bin \
    0x10000 xiaozhi-esp32s3-korvo1-v5.bin
```

### 使用ESP-IDF烧录
```bash
idf.py -p COM端口 flash
```

## 功能特性

### 已实现功能
- ✅ Wi-Fi连接
- ✅ 离线语音唤醒 (ESP-SR)
- ✅ 音频播放和录音
- ✅ 三阵列麦克风支持
- ✅ 按钮控制
- ✅ MCP设备控制

### 音频特性
- 采样率: 24kHz
- 编解码器: ES8311 (播放) + ES7210 (三阵列麦克风)
- 支持回声消除 (AEC)
- 支持声纹识别
- 支持声源定位
- 支持远场语音识别

### 麦克风特性
- 三阵列麦克风设计
- 支持多通道音频输入
- 支持降噪和回声消除
- 支持声源定位算法

## 使用说明

### 首次使用
1. 烧录固件到开发板
2. 上电后，通过串口查看启动信息
3. 长按BOOT按钮进入Wi-Fi配置模式
4. 连接Wi-Fi网络
5. 开始语音交互

### 按钮功能
- **单击**: 切换聊天状态
- **双击**: 切换AEC模式 (如果启用)
- **长按**: 重置Wi-Fi配置 (在启动时)

### 语音交互
- 说出唤醒词激活设备
- 进行语音对话
- 支持多轮对话
- 支持设备控制 (通过MCP)
- 支持远场语音识别
- 支持声源定位

## 故障排除

### 常见问题

**音频无输出**
- 检查I2S连接
- 确认ES8311地址正确
- 检查PA使能引脚

**麦克风无输入**
- 检查ES7210三阵列麦克风连接
- 确认I2C地址正确
- 检查麦克风电源供应

**Wi-Fi连接失败**
- 检查Wi-Fi凭据
- 确认网络可用
- 检查天线连接

**声源定位不准确**
- 检查三阵列麦克风安装位置
- 确认麦克风间距配置
- 检查音频处理算法参数

## 技术规格

- **工作电压**: 3.3V
- **工作温度**: -40°C ~ 85°C
- **存储温度**: -40°C ~ 125°C
- **相对湿度**: 10% ~ 90% (无凝露)

## 参考资料

- [ESP32-S3技术参考手册](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf)
- [ESP32-S3-KORVO1-V5开发板用户指南](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-korvo1-v3/user-guide.html)
- [小智AI项目文档](https://github.com/78/xiaozhi-esp32)

## 许可证

本项目采用MIT许可证，详见LICENSE文件。 