#ifndef MULTI_BUTTON_H_
#define MULTI_BUTTON_H_

#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <functional>
#include <vector>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

class MultiButton {
public:
    struct ButtonConfig {
        uint8_t button_id;
        uint16_t adc_min;
        uint16_t adc_max;
        const char* name;
    };

    MultiButton(gpio_num_t gpio_pin, const std::vector<ButtonConfig>& button_configs);
    ~MultiButton();

    void OnButtonPress(uint8_t button_id, std::function<void()> callback);
    void OnButtonLongPress(uint8_t button_id, std::function<void()> callback);
    void OnButtonDoubleClick(uint8_t button_id, std::function<void()> callback);
    
    // 静态方法，用于获取ADC值（供校准任务使用）
    static uint16_t GetAdcValue();
    static adc_oneshot_unit_handle_t GetAdcHandle() { return adc_handle_; }

private:
    gpio_num_t gpio_pin_;
    std::vector<ButtonConfig> button_configs_;
    std::vector<std::function<void()>> press_callbacks_;
    std::vector<std::function<void()>> long_press_callbacks_;
    std::vector<std::function<void()>> double_click_callbacks_;
    
    TimerHandle_t scan_timer_;
    TimerHandle_t debounce_timer_;
    
    uint8_t current_button_id_;
    uint8_t last_button_id_;
    uint32_t press_start_time_;
    uint32_t last_click_time_;
    uint8_t click_count_;
    bool button_pressed_;
    uint8_t release_count_;  // 添加释放计数器
    
    static void ScanTimerCallback(TimerHandle_t timer);
    static void DebounceTimerCallback(TimerHandle_t timer);
    
    void ScanButtons();
    uint8_t GetButtonIdFromAdc(uint16_t adc_value);
    uint16_t ReadAdc();
    void HandleButtonPress(uint8_t button_id);
    void HandleButtonRelease();
    
    static adc_oneshot_unit_handle_t adc_handle_;
    static SemaphoreHandle_t adc_mutex_;
};

#endif // MULTI_BUTTON_H_ 