#include "multi_button.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_adc/adc_oneshot.h>

#define TAG "MultiButton"

// 常量定义
#define SCAN_INTERVAL_MS 100  // 增加扫描间隔，减少重复触发
#define DEBOUNCE_TIME_MS 100  // 增加防抖时间
#define LONG_PRESS_TIME_MS 2000
#define DOUBLE_CLICK_TIME_MS 300
#define ADC_SAMPLE_COUNT 5
#define BUTTON_RELEASE_THRESHOLD 3  // 连续3次检测到释放才认为真正释放

// 静态成员变量定义
adc_oneshot_unit_handle_t MultiButton::adc_handle_ = nullptr;
SemaphoreHandle_t MultiButton::adc_mutex_ = nullptr;

MultiButton::MultiButton(gpio_num_t gpio_pin, const std::vector<ButtonConfig>& button_configs)
    : gpio_pin_(gpio_pin), button_configs_(button_configs), current_button_id_(0), 
      last_button_id_(0), press_start_time_(0), last_click_time_(0), 
      click_count_(0), button_pressed_(false), release_count_(0) {
    
    // 初始化回调数组
    press_callbacks_.resize(button_configs.size());
    long_press_callbacks_.resize(button_configs.size());
    double_click_callbacks_.resize(button_configs.size());
    
    // 配置GPIO为ADC输入
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << gpio_pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    
    // 配置ADC - 使用新的ADC API（只在第一次初始化）
    if (adc_handle_ == nullptr) {
        adc_oneshot_unit_init_cfg_t init_config1 = {
            .unit_id = ADC_UNIT_1,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_handle_));
        
        adc_oneshot_chan_cfg_t chan_config = {
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle_, ADC_CHANNEL_7, &chan_config));
        
        // 创建ADC互斥锁
        adc_mutex_ = xSemaphoreCreateMutex();
        if (adc_mutex_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create ADC mutex");
        }
    }
    
    // 创建扫描定时器
    scan_timer_ = xTimerCreate("button_scan", pdMS_TO_TICKS(SCAN_INTERVAL_MS), 
                              pdTRUE, this, ScanTimerCallback);
    
    // 创建防抖定时器
    debounce_timer_ = xTimerCreate("button_debounce", pdMS_TO_TICKS(DEBOUNCE_TIME_MS), 
                                  pdFALSE, this, DebounceTimerCallback);
    
    // 启动扫描定时器
    xTimerStart(scan_timer_, 0);
    
    ESP_LOGI(TAG, "MultiButton initialized with %d buttons on GPIO %d", 
             button_configs.size(), gpio_pin);
}

MultiButton::~MultiButton() {
    if (scan_timer_) {
        xTimerDelete(scan_timer_, 0);
    }
    if (debounce_timer_) {
        xTimerDelete(debounce_timer_, 0);
    }
    // 注意：不在这里删除ADC句柄，因为它是静态的，可能被其他实例使用
}

void MultiButton::OnButtonPress(uint8_t button_id, std::function<void()> callback) {
    if (button_id < press_callbacks_.size()) {
        press_callbacks_[button_id] = callback;
    }
}

void MultiButton::OnButtonLongPress(uint8_t button_id, std::function<void()> callback) {
    if (button_id < long_press_callbacks_.size()) {
        long_press_callbacks_[button_id] = callback;
    }
}

void MultiButton::OnButtonDoubleClick(uint8_t button_id, std::function<void()> callback) {
    if (button_id < double_click_callbacks_.size()) {
        double_click_callbacks_[button_id] = callback;
    }
}

void MultiButton::ScanTimerCallback(TimerHandle_t timer) {
    MultiButton* button = static_cast<MultiButton*>(pvTimerGetTimerID(timer));
    button->ScanButtons();
}

void MultiButton::DebounceTimerCallback(TimerHandle_t timer) {
    MultiButton* button = static_cast<MultiButton*>(pvTimerGetTimerID(timer));
    button->HandleButtonRelease();
}

void MultiButton::ScanButtons() {
    uint16_t adc_value = ReadAdc();
    uint8_t button_id = GetButtonIdFromAdc(adc_value);
    
    if (button_id > 0 && !button_pressed_) {
        // 检测到按键按下
        current_button_id_ = button_id;
        press_start_time_ = esp_timer_get_time() / 1000; // 转换为毫秒
        button_pressed_ = true;
        release_count_ = 0; // 重置释放计数器
        
        ESP_LOGI(TAG, "Button %d pressed (ADC: %d)", button_id, adc_value);
        
        // 启动防抖定时器
        xTimerStart(debounce_timer_, 0);
        
    } else if (button_id == 0 && button_pressed_) {
        // 检测到可能的按键释放，增加计数器
        release_count_++;
        if (release_count_ >= BUTTON_RELEASE_THRESHOLD) {
            // 连续多次检测到释放，确认按键已释放
            ESP_LOGI(TAG, "Button %d released (confirmed)", current_button_id_);
            HandleButtonRelease();
            release_count_ = 0;
        }
    } else if (button_pressed_ && button_id == current_button_id_) {
        // 按键仍然按下，重置释放计数器
        release_count_ = 0;
        
        // 检查长按
        uint32_t press_duration = (esp_timer_get_time() / 1000) - press_start_time_;
        if (press_duration >= LONG_PRESS_TIME_MS) {
            if (long_press_callbacks_[current_button_id_ - 1]) {
                ESP_LOGI(TAG, "Button %d long pressed", current_button_id_);
                long_press_callbacks_[current_button_id_ - 1]();
                button_pressed_ = false; // 防止重复触发
                release_count_ = 0;
            }
        }
    } else if (button_pressed_ && button_id != current_button_id_ && button_id > 0) {
        // 检测到不同的按键，可能是误触，重置状态
        ESP_LOGW(TAG, "Button changed from %d to %d, resetting", current_button_id_, button_id);
        button_pressed_ = false;
        release_count_ = 0;
        current_button_id_ = 0;
    }
}

uint8_t MultiButton::GetButtonIdFromAdc(uint16_t adc_value) {
    for (const auto& config : button_configs_) {
        if (adc_value >= config.adc_min && adc_value <= config.adc_max) {
            return config.button_id;
        }
    }
    return 0; // 没有按键被按下
}

uint16_t MultiButton::ReadAdc() {
    if (adc_handle_ == nullptr || adc_mutex_ == nullptr) {
        return 4095; // 默认值表示没有按键按下
    }
    
    // 尝试获取互斥锁
    if (xSemaphoreTake(adc_mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire ADC mutex");
        return 4095; // 默认值表示没有按键按下
    }
    
    uint32_t sum = 0;
    int adc_raw = 0;
    bool read_success = true;
    
    for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
        esp_err_t ret = adc_oneshot_read(adc_handle_, ADC_CHANNEL_7, &adc_raw);
        if (ret == ESP_OK) {
            sum += adc_raw;
        } else {
            ESP_LOGW(TAG, "ADC read failed: %s", esp_err_to_name(ret));
            read_success = false;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2)); // 增加延迟时间
    }
    
    // 释放互斥锁
    xSemaphoreGive(adc_mutex_);
    
    if (read_success) {
        return sum / ADC_SAMPLE_COUNT;
    } else {
        return 4095; // 默认值表示没有按键按下
    }
}

uint16_t MultiButton::GetAdcValue() {
    if (adc_handle_ == nullptr || adc_mutex_ == nullptr) {
        return 4095; // 默认值表示没有按键按下
    }
    
    // 尝试获取互斥锁
    if (xSemaphoreTake(adc_mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire ADC mutex");
        return 4095; // 默认值表示没有按键按下
    }
    
    uint32_t sum = 0;
    int adc_raw = 0;
    bool read_success = true;
    
    for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
        esp_err_t ret = adc_oneshot_read(adc_handle_, ADC_CHANNEL_7, &adc_raw);
        if (ret == ESP_OK) {
            sum += adc_raw;
        } else {
            ESP_LOGW(TAG, "ADC read failed: %s", esp_err_to_name(ret));
            read_success = false;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2)); // 增加延迟时间
    }
    
    // 释放互斥锁
    xSemaphoreGive(adc_mutex_);
    
    if (read_success) {
        return sum / ADC_SAMPLE_COUNT;
    } else {
        return 4095; // 默认值表示没有按键按下
    }
}

void MultiButton::HandleButtonPress(uint8_t button_id) {
    if (button_id > 0 && button_id <= press_callbacks_.size()) {
        if (press_callbacks_[button_id - 1]) {
            press_callbacks_[button_id - 1]();
        }
    }
}

void MultiButton::HandleButtonRelease() {
    if (button_pressed_) {
        uint32_t current_time = esp_timer_get_time() / 1000;
        uint32_t press_duration = current_time - press_start_time_;
        
        if (press_duration >= DEBOUNCE_TIME_MS && press_duration < LONG_PRESS_TIME_MS) {
            // 短按 - 确保只触发一次
            ESP_LOGI(TAG, "Button %d short press confirmed (duration: %lu ms)", 
                     current_button_id_, press_duration);
            HandleButtonPress(current_button_id_);
            
            // 检查双击
            if (current_time - last_click_time_ < DOUBLE_CLICK_TIME_MS && 
                last_button_id_ == current_button_id_) {
                click_count_++;
                if (click_count_ == 2) {
                    ESP_LOGI(TAG, "Button %d double click detected", current_button_id_);
                    if (double_click_callbacks_[current_button_id_ - 1]) {
                        double_click_callbacks_[current_button_id_ - 1]();
                    }
                    click_count_ = 0;
                }
            } else {
                click_count_ = 1;
            }
            
            last_click_time_ = current_time;
            last_button_id_ = current_button_id_;
                 } else if (press_duration < DEBOUNCE_TIME_MS) {
             ESP_LOGD(TAG, "Button %d press too short, ignored (duration: %lu ms)", 
                      current_button_id_, press_duration);
         }
        
        // 重置状态
        button_pressed_ = false;
        current_button_id_ = 0;
        release_count_ = 0;
    }
} 