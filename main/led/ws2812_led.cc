#include "ws2812_led.h"
#include <esp_log.h>
#include <math.h>

#define TAG "WS2812Led"

WS2812Led::WS2812Led(gpio_num_t gpio, uint16_t led_count) 
    : mutex_(), gpio_(gpio), led_count_(led_count), brightness_(255), is_on_(false), 
      led_strip_(nullptr), led_colors_(nullptr), current_effect_state_(LedEffectState::IDLE), 
      effect_state_changed_(false), effect_task_(nullptr), voice_effect_active_(false), 
      is_input_mode_(true), voice_level_(0), voice_effect_task_(nullptr), 
      breathing_effect_active_(false), breathing_effect_task_(nullptr) {
    
    // 分配LED颜色数组
    led_colors_ = new WS2812Color[led_count_];
    if (led_colors_) {
        // 初始化所有LED为黑色（关闭状态）
        for (uint16_t i = 0; i < led_count_; i++) {
            led_colors_[i] = WS2812Color::Black();
        }
    }
    
    ESP_LOGI(TAG, "WS2812 LED driver initialized: GPIO=%d, LED count=%d", gpio_, led_count_);
}

WS2812Led::~WS2812Led() {
    // 停止效果任务
    if (effect_task_) {
        vTaskDelete(effect_task_);
        effect_task_ = nullptr;
    }
    
    // 停止语音效果任务
    if (voice_effect_task_) {
        vTaskDelete(voice_effect_task_);
        voice_effect_task_ = nullptr;
    }
    
    // 停止呼吸灯效果任务
    if (breathing_effect_task_) {
        vTaskDelete(breathing_effect_task_);
        breathing_effect_task_ = nullptr;
    }
    
    // 清理资源
    DeinitLedStrip();
    
    if (led_colors_) {
        delete[] led_colors_;
        led_colors_ = nullptr;
    }
}

esp_err_t WS2812Led::InitLedStrip() {
    // LED strip配置
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = gpio_;
    strip_config.max_leds = led_count_;
    strip_config.led_pixel_format = LED_PIXEL_FORMAT_GRB;
    strip_config.led_model = LED_MODEL_WS2812;
    
    // RMT配置
    led_strip_rmt_config_t rmt_config = {};
    rmt_config.resolution_hz = 10 * 1000 * 1000; // 10MHz
    
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip_));
    led_strip_clear(led_strip_);
    
    ESP_LOGI(TAG, "LED strip initialized for WS2812");
    return ESP_OK;
}

void WS2812Led::DeinitLedStrip() {
    if (led_strip_) {
        led_strip_del(led_strip_);
        led_strip_ = nullptr;
    }
}

void WS2812Led::OnStateChanged() {
    // 这里可以根据设备状态改变LED效果
    // 暂时不立即点亮LED，等待具体的状态变化
    ESP_LOGI(TAG, "WS2812 LED state changed - ready for control");
}

void WS2812Led::TurnOn() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    ESP_LOGI(TAG, "TurnOn called, is_on_=%d, led_strip_=%p", is_on_, led_strip_);
    
    if (!is_on_) {
        // 初始化LED strip（如果还没有初始化）
        if (!led_strip_) {
            ESP_LOGI(TAG, "Initializing LED strip...");
            InitLedStrip();
        }
        
        // 点亮所有LED为白色
        ESP_LOGI(TAG, "Setting all LEDs to white...");
        SetAllLeds(WS2812Color::White());
        is_on_ = true;
        ESP_LOGI(TAG, "WS2812 LEDs turned ON");
    }
}

void WS2812Led::TurnOff() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    ESP_LOGI(TAG, "TurnOff called, is_on_=%d, led_strip_=%p", is_on_, led_strip_);
    
    if (is_on_) {
        // 关闭所有LED
        if (led_strip_) {
            ESP_LOGI(TAG, "Clearing LED strip...");
            led_strip_clear(led_strip_);
        }
        is_on_ = false;
        ESP_LOGI(TAG, "WS2812 LEDs turned OFF");
    }
}

void WS2812Led::SetBrightness(uint8_t brightness) {
    std::lock_guard<std::mutex> lock(mutex_);
    brightness_ = brightness;
    ESP_LOGI(TAG, "Brightness set to %d", brightness_);
}

void WS2812Led::SetLedColor(uint16_t led_index, const WS2812Color& color) {
    if (led_index >= led_count_) {
        ESP_LOGW(TAG, "Invalid LED index: %d (max: %d)", led_index, led_count_ - 1);
        return;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    led_colors_[led_index] = color;
    UpdateLeds();
}

void WS2812Led::SetAllLeds(const WS2812Color& color) {
    ESP_LOGI(TAG, "SetAllLeds called with color R=%d, G=%d, B=%d", color.r, color.g, color.b);
    
    ESP_LOGI(TAG, "Setting all %d LEDs to the same color", led_count_);
    
    for (uint16_t i = 0; i < led_count_; i++) {
        led_colors_[i] = color;
    }
    
    ESP_LOGI(TAG, "All LEDs set in memory, calling UpdateLeds...");
    UpdateLeds();
    ESP_LOGI(TAG, "SetAllLeds completed");
}

void WS2812Led::UpdateLeds() {
    ESP_LOGI(TAG, "UpdateLeds called, brightness_=%d, led_count_=%d", brightness_, led_count_);
    
    if (!led_strip_) {
        ESP_LOGW(TAG, "LED strip not initialized, cannot update LEDs");
        return;
    }
    
    ESP_LOGI(TAG, "LED strip is initialized, proceeding with update");
    
    // 设置所有LED的颜色
    for (uint16_t i = 0; i < led_count_; i++) {
        uint8_t r = (led_colors_[i].r * brightness_) / 255;
        uint8_t g = (led_colors_[i].g * brightness_) / 255;
        uint8_t b = (led_colors_[i].b * brightness_) / 255;
        
        ESP_LOGI(TAG, "Setting LED %d to R=%d, G=%d, B=%d", i, r, g, b);
        esp_err_t ret = led_strip_set_pixel(led_strip_, i, r, g, b);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set LED %d: %s", i, esp_err_to_name(ret));
        }
    }
    
    // 刷新显示
    ESP_LOGI(TAG, "Refreshing LED strip...");
    esp_err_t ret = led_strip_refresh(led_strip_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to refresh LED strip: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "LED strip refresh completed successfully");
    }
}

void WS2812Led::SetEffectState(LedEffectState new_state) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (current_effect_state_ != new_state) {
        ESP_LOGI(TAG, "Effect state changing from %d to %d", 
                 static_cast<int>(current_effect_state_), static_cast<int>(new_state));
        
        // 停止当前效果
        StopAllEffects();
        
        // 更新状态
        current_effect_state_ = new_state;
        // 重置状态变化标志，因为StopAllEffects已经设置了它
        effect_state_changed_ = false;
        
        ESP_LOGI(TAG, "Effect state changed to %d", static_cast<int>(new_state));
    } else {
        // 如果状态没有变化，重置状态变化标志
        effect_state_changed_ = false;
    }
}

void WS2812Led::StopAllEffects() {
    ESP_LOGI(TAG, "Stopping all effects");
    
    // Signal all tasks to stop
    voice_effect_active_ = false;
    breathing_effect_active_ = false;
    effect_state_changed_ = true;
    
    // Wait for tasks to self-terminate
    const int max_wait_ticks = pdMS_TO_TICKS(100); // 100ms timeout
    int wait_count = 0;
    
    // Wait for voice effect task to stop
    while (voice_effect_task_ != nullptr && wait_count < max_wait_ticks) {
        vTaskDelay(pdMS_TO_TICKS(1));
        wait_count++;
    }
    
    // Force delete voice effect task if it didn't stop
    if (voice_effect_task_ != nullptr) {
        vTaskDelete(voice_effect_task_);
        voice_effect_task_ = nullptr;
    }
    
    // Reset wait counter for breathing effect
    wait_count = 0;
    
    // Wait for breathing effect task to stop
    while (breathing_effect_task_ != nullptr && wait_count < max_wait_ticks) {
        vTaskDelay(pdMS_TO_TICKS(1));
        wait_count++;
    }
    
    // Force delete breathing effect task if it didn't stop
    if (breathing_effect_task_ != nullptr) {
        vTaskDelete(breathing_effect_task_);
        breathing_effect_task_ = nullptr;
    }
    
    // Stop other effects
    if (effect_task_) {
        vTaskDelete(effect_task_);
        effect_task_ = nullptr;
    }
    
    // Clear LED display
    if (led_strip_) {
        led_strip_clear(led_strip_);
    }
}

bool WS2812Led::CanStartEffect(LedEffectState effect) const {
    // 语音效果具有最高优先级，可以覆盖其他效果
    if (effect == LedEffectState::VOICE_INPUT || effect == LedEffectState::VOICE_OUTPUT) {
        return true;
    }
    
    // 其他效果只能在空闲状态或呼吸灯状态下启动
    return (current_effect_state_ == LedEffectState::IDLE || 
            current_effect_state_ == LedEffectState::BREATHING);
}

uint32_t WS2812Led::ColorToRgb(const WS2812Color& color) {
    return (color.r << 16) | (color.g << 8) | color.b;
}

void WS2812Led::TestTurnOn() {
    ESP_LOGI(TAG, "TestTurnOn called - LED strip ready for effects");
    // 不自动点亮LED，等待具体的效果控制
}

void WS2812Led::Rainbow(uint16_t start_index, uint16_t end_index) {
    if (end_index == 0) {
        end_index = led_count_;
    }
    
    if (start_index >= led_count_ || end_index > led_count_ || start_index >= end_index) {
        ESP_LOGW(TAG, "Invalid rainbow range: start=%d, end=%d", start_index, end_index);
        return;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    uint16_t led_count_in_range = end_index - start_index;
    for (uint16_t i = 0; i < led_count_in_range; i++) {
        uint16_t led_index = start_index + i;
        float hue = (float)i / led_count_in_range * 360.0f;
        
        // HSV to RGB转换
        float h = hue / 60.0f;
        int hi = (int)h % 6;
        
        float v = 1.0f;
        float s = 1.0f;
        
        float c = v * s;
        float x = c * (1 - fabs(fmod(h, 2) - 1));
        float m = v - c;
        
        float r, g, b;
        switch (hi) {
            case 0: r = c; g = x; b = 0; break;
            case 1: r = x; g = c; b = 0; break;
            case 2: r = 0; g = c; b = x; break;
            case 3: r = 0; g = x; b = c; break;
            case 4: r = x; g = 0; b = c; break;
            case 5: r = c; g = 0; b = x; break;
            default: r = 0; g = 0; b = 0; break;
        }
        
        led_colors_[led_index] = WS2812Color(
            (uint8_t)((r + m) * 255),
            (uint8_t)((g + m) * 255),
            (uint8_t)((b + m) * 255)
        );
    }
    
    UpdateLeds();
}

void WS2812Led::FadeIn(const WS2812Color& color, uint32_t duration_ms) {
    StartEffectTask(FadeTask);
}

void WS2812Led::FadeOut(uint32_t duration_ms) {
    FadeIn(WS2812Color::Black(), duration_ms);
}

void WS2812Led::StartEffectTask(void (*effect_func)(void*)) {
    // 停止现有效果任务
    if (effect_task_) {
        vTaskDelete(effect_task_);
        effect_task_ = nullptr;
    }
    
    // 创建新效果任务
    xTaskCreate(effect_func, "ws2812_effect", 4096, this, 5, &effect_task_);
}

void WS2812Led::RainbowTask(void* param) {
    WS2812Led* led = static_cast<WS2812Led*>(param);
    
    while (true) {
        led->Rainbow();
        vTaskDelay(pdMS_TO_TICKS(50)); // 20fps
    }
}

void WS2812Led::FadeTask(void* param) {
    WS2812Led* led = static_cast<WS2812Led*>(param);
    
    // 简单的淡入效果
    for (int brightness = 0; brightness <= 255; brightness += 5) {
        led->SetBrightness(brightness);
        led->UpdateLeds();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    // 任务结束，删除自己
    vTaskDelete(nullptr);
} 

void WS2812Led::StartVoiceInputEffect() {
    ESP_LOGI(TAG, "Starting voice input effect");
    
    // 检查是否可以启动语音效果
    if (!CanStartEffect(LedEffectState::VOICE_INPUT)) {
        ESP_LOGW(TAG, "Cannot start voice input effect in current state %d", 
                 static_cast<int>(current_effect_state_));
        return;
    }
    
    // 设置状态
    SetEffectState(LedEffectState::VOICE_INPUT);
    
    is_input_mode_ = true;
    voice_effect_active_ = true;
    voice_level_ = 0;
    
    // 启动语音效果任务
    xTaskCreate(VoiceEffectTask, "voice_effect", 4096, this, 5, &voice_effect_task_);
}

void WS2812Led::StartVoiceOutputEffect() {
    ESP_LOGI(TAG, "Starting voice output effect");
    
    // 检查是否可以启动语音效果
    if (!CanStartEffect(LedEffectState::VOICE_OUTPUT)) {
        ESP_LOGW(TAG, "Cannot start voice output effect in current state %d", 
                 static_cast<int>(current_effect_state_));
        return;
    }
    
    // 设置状态
    SetEffectState(LedEffectState::VOICE_OUTPUT);
    
    is_input_mode_ = false;
    voice_effect_active_ = true;
    voice_level_ = 0;
    
    // 启动语音效果任务
    xTaskCreate(VoiceEffectTask, "voice_effect", 4096, this, 5, &voice_effect_task_);
}

void WS2812Led::StopVoiceEffect() {
    ESP_LOGI(TAG, "Stopping voice effect");
    
    // 如果当前是语音效果状态，则回到空闲状态
    if (current_effect_state_ == LedEffectState::VOICE_INPUT || 
        current_effect_state_ == LedEffectState::VOICE_OUTPUT) {
        SetEffectState(LedEffectState::IDLE);
    }
}

void WS2812Led::UpdateVoiceLevel(uint8_t level) {
    voice_level_ = level;
}

void WS2812Led::VoiceEffectTask(void* param) {
    WS2812Led* led = static_cast<WS2812Led*>(param);
    
    // 确保LED strip已初始化
    if (!led->led_strip_) {
        led->InitLedStrip();
    }
    
    ESP_LOGI(TAG, "VoiceEffectTask started, is_input_mode=%d", led->is_input_mode_);
    
    while (led->voice_effect_active_) {
        // 检查状态是否发生变化
        if (led->effect_state_changed_) {
            ESP_LOGI(TAG, "VoiceEffectTask detected state change, stopping");
            break;
        }
        
        // 检查是否应该停止
        if (!led->voice_effect_active_) {
            break;
        }
        
        uint8_t level = led->voice_level_;
        bool is_input = led->is_input_mode_;
        
        // 计算最大距离（从中心到最远处的LED）
        uint16_t max_distance = led->led_count_ / 2;
        
        // 根据音量级别计算活跃范围，确保音量最大时能覆盖所有LED
        uint16_t active_leds = (level * max_distance) / 255;
        if (active_leds > max_distance) active_leds = max_distance;
        
        // 确保即使音量很低也有最小效果，并且最远处LED也能被点亮
        if (active_leds == 0 && level > 5) {
            active_leds = 1;
        }
        
        // 确保音量最大时能覆盖所有LED（包括最远处的LED）
        if (level >= 200) {  // 当音量达到80%以上时
            active_leds = max_distance;  // 覆盖所有LED
        }
        
        // ESP_LOGI(TAG, "VoiceEffectTask: level=%d, active_leds=%d, max_distance=%d, is_input=%d", 
                //  level, active_leds, max_distance, is_input);
        
        // 清除所有LED
        led_strip_clear(led->led_strip_);
        
        // 添加小延迟让RMT通道稳定
        vTaskDelay(pdMS_TO_TICKS(1));
        
        // 计算中心位置
        uint16_t center_left = led->led_count_ / 2 - 1;  // 左中心
        uint16_t center_right = led->led_count_ / 2;     // 右中心
        
        // 从中心向两边扩散的效果
        for (uint16_t i = 0; i < led->led_count_; i++) {
            uint8_t brightness = 0;
            
            // 计算到中心的距离
            uint16_t distance_from_center;
            if (i < center_right) {
                // 左半边
                distance_from_center = center_left - i;
            } else {
                // 右半边
                distance_from_center = i - center_right;
            }
            
            // 如果距离在活跃范围内，计算亮度
            if (distance_from_center < active_leds) {
                // 根据距离计算亮度，越靠近中心越亮（使用固定的最大距离，确保亮度不变化）
                uint8_t position_brightness = 255 - ((distance_from_center * 255) / max_distance);
                brightness = position_brightness;  // 亮度固定，不随active_leds变化
                
            }
            
            // 设置颜色（用户输入用蓝色，系统输出用绿色）
            uint8_t r, g, b;
            if (is_input) {
                r = 0; g = 100; b = 255; // 蓝色
            } else {
                r = 0; g = 255; b = 100; // 绿色
            }
            
            // 应用亮度
            r = (r * brightness) / 255;
            g = (g * brightness) / 255;
            b = (b * brightness) / 255;
            
            led_strip_set_pixel(led->led_strip_, i, r, g, b);
        }
        
        // 刷新LED strip，添加错误处理
        esp_err_t ret = led_strip_refresh(led->led_strip_);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to refresh LED strip: %s", esp_err_to_name(ret));
            // 如果刷新失败，尝试重新初始化LED strip
            if (ret == ESP_ERR_INVALID_STATE) {
                ESP_LOGI(TAG, "Reinitializing LED strip due to RMT error");
                led->DeinitLedStrip();
                vTaskDelay(pdMS_TO_TICKS(10)); // 等待RMT通道重置
                led->InitLedStrip();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // 20fps
    }
    
    ESP_LOGI(TAG, "VoiceEffectTask stopped");
    // 任务结束，删除自己
    vTaskDelete(nullptr);
}

void WS2812Led::StartBreathingEffect() {
    ESP_LOGI(TAG, "Starting breathing effect");
    
    // 检查是否可以启动呼吸灯效果
    if (!CanStartEffect(LedEffectState::BREATHING)) {
        ESP_LOGW(TAG, "Cannot start breathing effect in current state %d", 
                 static_cast<int>(current_effect_state_));
        return;
    }
    
    // 设置状态
    SetEffectState(LedEffectState::BREATHING);
    
    breathing_effect_active_ = true;
    
    // 启动呼吸灯效果任务
    xTaskCreate(BreathingEffectTask, "breathing_effect", 4096, this, 5, &breathing_effect_task_);
}

void WS2812Led::StopBreathingEffect() {
    ESP_LOGI(TAG, "Stopping breathing effect");
    
    // 如果当前是呼吸灯效果状态，则回到空闲状态
    if (current_effect_state_ == LedEffectState::BREATHING) {
        SetEffectState(LedEffectState::IDLE);
    }
}

void WS2812Led::BreathingEffectTask(void* param) {
    WS2812Led* led = static_cast<WS2812Led*>(param);
    
    // 确保LED strip已初始化
    if (!led->led_strip_) {
        led->InitLedStrip();
    }
    
    ESP_LOGI(TAG, "BreathingEffectTask started");
    
    uint8_t brightness = 0;
    int8_t direction = 1; // 1表示变亮，-1表示变暗
    
    while (led->breathing_effect_active_) {
        // 检查状态是否发生变化
        if (led->effect_state_changed_) {
            ESP_LOGI(TAG, "BreathingEffectTask detected state change, stopping");
            break;
        }
        
        // 检查是否应该停止
        if (!led->breathing_effect_active_) {
            break;
        }
        
        // 清除所有LED
        led_strip_clear(led->led_strip_);
        
        // 添加小延迟让RMT通道稳定
        vTaskDelay(pdMS_TO_TICKS(1));
        
        // 设置所有LED为白色，应用呼吸效果
        for (uint16_t i = 0; i < led->led_count_; i++) {
            uint8_t r = (255 * brightness) / 255;
            uint8_t g = (255 * brightness) / 255;
            uint8_t b = (255 * brightness) / 255;
            led_strip_set_pixel(led->led_strip_, i, r, g, b);
        }
        
        // 刷新LED strip
        esp_err_t ret = led_strip_refresh(led->led_strip_);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to refresh LED strip: %s", esp_err_to_name(ret));
            // 如果刷新失败，尝试重新初始化LED strip
            if (ret == ESP_ERR_INVALID_STATE) {
                ESP_LOGI(TAG, "Reinitializing LED strip due to RMT error");
                led->DeinitLedStrip();
                vTaskDelay(pdMS_TO_TICKS(10)); // 等待RMT通道重置
                led->InitLedStrip();
            }
        }
        
        // 更新亮度
        brightness += direction * 5;
        
        // 检查亮度边界
        if (brightness >= 255) {
            brightness = 255;
            direction = -1; // 开始变暗
        } else if (brightness <= 0) {
            brightness = 0;
            direction = 1; // 开始变亮
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); // 20fps
    }
    
    ESP_LOGI(TAG, "BreathingEffectTask stopped");
    vTaskDelete(nullptr);
} 

/*
使用示例：

1. 在idle模式下启动呼吸灯效果：
   led->StartBreathingEffect();

2. 当进入speaking模式时，语音效果会自动覆盖呼吸灯效果：
   led->StartVoiceOutputEffect();

3. 当进入listening模式时，语音效果会自动覆盖呼吸灯效果：
   led->StartVoiceInputEffect();

4. 当语音模式结束时，可以重新启动呼吸灯效果：
   led->StopVoiceEffect();
   led->StartBreathingEffect();

5. 检查当前状态：
   LedEffectState state = led->GetCurrentEffectState();
   if (state == LedEffectState::IDLE) {
       // 可以启动任何效果
   } else if (state == LedEffectState::BREATHING) {
       // 语音效果可以覆盖呼吸灯效果
   }

状态优先级：
- VOICE_INPUT/VOICE_OUTPUT: 最高优先级，可以覆盖其他所有效果
- BREATHING: 中等优先级，只能在IDLE状态下启动
- IDLE: 最低优先级，可以启动任何效果
*/ 