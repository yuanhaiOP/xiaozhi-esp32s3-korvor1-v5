#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "multi_button.h"

#define TAG "ADCCalibration"

void adc_calibration_task(void* param) {
    ESP_LOGI(TAG, "Starting ADC calibration for GPIO8 (ADC_CHANNEL_7)");
    
    // 等待MultiButton初始化完成
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 检查MultiButton是否已经初始化了ADC
    if (MultiButton::GetAdcHandle() == nullptr) {
        ESP_LOGE(TAG, "MultiButton not initialized yet, cannot calibrate ADC");
        vTaskDelete(nullptr);
        return;
    }
    
    ESP_LOGI(TAG, "Using MultiButton's ADC handle for calibration");
    
    ESP_LOGI(TAG, "ADC calibration started. Press buttons and observe ADC values:");
    ESP_LOGI(TAG, "No button pressed should show ~4095");
    ESP_LOGI(TAG, "Each button should show different values");
    ESP_LOGI(TAG, "Press Ctrl+C to stop");
    
    uint16_t last_value = 0;
    uint32_t stable_count = 0;
    
    while (true) {
        uint16_t current_value = MultiButton::GetAdcValue();
        
        // 只在值发生变化时打印
        if (current_value != last_value) {
            // 判断当前ADC值对应的按键（与实际配置保持一致）
            const char* button_name = "None";
            if (current_value >= 300 && current_value <= 500) {
                button_name = "Button1 (Toggle chat)";
            } else if (current_value >= 2700 && current_value <= 2900) {
                button_name = "Button2 (Volume up)";
            } else if (current_value >= 2000 && current_value <= 2400) {
                button_name = "Button3 (Volume down)";
            } else if (current_value >= 800 && current_value <= 1000) {
                button_name = "Button4 (Toggle LED)";
            } else if (current_value >= 1200 && current_value <= 1400) {
                button_name = "Button5 (Reset WiFi)";
            } else if (current_value >= 1800 && current_value <= 1900) {
                button_name = "Button6 (Toggle AEC)";
            }
            
            ESP_LOGI(TAG, "ADC Value: %d -> %s", current_value, button_name);
            last_value = current_value;
            stable_count = 0;
        } else {
            stable_count++;
            if (stable_count % 100 == 0) {  // 每100次稳定值打印一次
                ESP_LOGI(TAG, "Stable ADC Value: %u (count: %lu)", current_value, stable_count);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));  // 5Hz采样率，减少与按键扫描的冲突
    }
}

void start_adc_calibration() {
    xTaskCreate(adc_calibration_task, "adc_calibration", 4096, nullptr, 5, nullptr);
}

// 清理ADC资源
void cleanup_adc_calibration() {
    // 这个函数可以在需要时添加清理逻辑
} 