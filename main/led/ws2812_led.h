#ifndef _WS2812_LED_H_
#define _WS2812_LED_H_

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "led.h"
#include <led_strip.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <atomic>
#include <mutex>

// WS2812 LED颜色结构体
struct WS2812Color {
    uint8_t r;  // 红色分量 (0-255)
    uint8_t g;  // 绿色分量 (0-255)
    uint8_t b;  // 蓝色分量 (0-255)
    
    WS2812Color() : r(0), g(0), b(0) {}
    WS2812Color(uint8_t red, uint8_t green, uint8_t blue) : r(red), g(green), b(blue) {}
    
    // 预定义颜色
    static WS2812Color Red() { return WS2812Color(255, 0, 0); }
    static WS2812Color Green() { return WS2812Color(0, 255, 0); }
    static WS2812Color Blue() { return WS2812Color(0, 0, 255); }
    static WS2812Color White() { return WS2812Color(255, 255, 255); }
    static WS2812Color Black() { return WS2812Color(0, 0, 0); }
    static WS2812Color Yellow() { return WS2812Color(255, 255, 0); }
    static WS2812Color Purple() { return WS2812Color(255, 0, 255); }
    static WS2812Color Cyan() { return WS2812Color(0, 255, 255); }
};

// LED效果状态枚举
enum class LedEffectState {
    IDLE = 0,           // 空闲状态
    BREATHING = 1,      // 呼吸灯效果
    VOICE_INPUT = 2,    // 语音输入效果
    VOICE_OUTPUT = 3,   // 语音输出效果
    RAINBOW = 4,        // 彩虹效果
    FADE = 5            // 淡入淡出效果
};

class WS2812Led : public Led {
public:
    WS2812Led(gpio_num_t gpio, uint16_t led_count = 12);
    virtual ~WS2812Led();

    void OnStateChanged() override;
    
    // 基本控制函数
    void TurnOn();
    void TurnOff();
    void SetBrightness(uint8_t brightness);
    
    // 单个LED控制
    void SetLedColor(uint16_t led_index, const WS2812Color& color);
    void SetAllLeds(const WS2812Color& color);
    
    // 特殊效果
    void Rainbow(uint16_t start_index = 0, uint16_t end_index = 0);
    void FadeIn(const WS2812Color& color, uint32_t duration_ms = 1000);
    void FadeOut(uint32_t duration_ms = 1000);
    
    // 语音驱动的LED效果
    void StartVoiceInputEffect();  // 用户输入语音时的效果
    void StartVoiceOutputEffect(); // 系统输出语音时的效果
    void StopVoiceEffect();        // 停止语音效果
    void UpdateVoiceLevel(uint8_t level); // 更新语音音量级别 (0-255)
    
    // 呼吸灯效果
    void StartBreathingEffect();   // 启动呼吸灯效果
    void StopBreathingEffect();    // 停止呼吸灯效果
    
    // 获取LED数量
    uint16_t GetLedCount() const { return led_count_; }
    
    // 获取当前效果状态
    LedEffectState GetCurrentEffectState() const { return current_effect_state_; }
    
    // 测试函数 - 手动点亮LED
    void TestTurnOn();

private:
    std::mutex mutex_;
    gpio_num_t gpio_;
    uint16_t led_count_;
    uint8_t brightness_;
    bool is_on_;
    
    led_strip_handle_t led_strip_;
    
    // LED颜色数组
    WS2812Color* led_colors_;
    
    // 状态管理
    LedEffectState current_effect_state_;
    std::atomic<bool> effect_state_changed_;
    
    // 初始化LED strip
    esp_err_t InitLedStrip();
    void DeinitLedStrip();
    
    // 更新LED显示
    void UpdateLeds();
    
    // 颜色转换
    uint32_t ColorToRgb(const WS2812Color& color);
    
    // 状态管理函数
    void SetEffectState(LedEffectState new_state);
    void StopAllEffects();
    bool CanStartEffect(LedEffectState effect) const;
    
    // 特殊效果任务
    TaskHandle_t effect_task_;
    void StartEffectTask(void (*effect_func)(void*));
    static void RainbowTask(void* param);
    static void FadeTask(void* param);
    
    // 语音效果相关
    bool voice_effect_active_;
    bool is_input_mode_;  // true=用户输入, false=系统输出
    uint8_t voice_level_;
    TaskHandle_t voice_effect_task_;
    static void VoiceEffectTask(void* param);
    
    // 呼吸灯效果相关
    bool breathing_effect_active_;
    TaskHandle_t breathing_effect_task_;
    static void BreathingEffectTask(void* param);
};

#endif  // _WS2812_LED_H_ 