#include "robot_movement_controller.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "RobotTest"

// 测试任务
void RobotTestTask(void* arg) {
    RobotMovementController* controller = static_cast<RobotMovementController*>(arg);
    
    ESP_LOGI(TAG, "Starting robot control test...");
    
    // 等待系统稳定
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 测试1: 前进
    ESP_LOGI(TAG, "Test 1: Forward movement");
    controller->ProcessVoiceCommand("前进");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // 测试2: 停止
    ESP_LOGI(TAG, "Test 2: Stop");
    controller->ProcessVoiceCommand("停止");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 测试3: 左转
    ESP_LOGI(TAG, "Test 3: Turn left");
    controller->ProcessVoiceCommand("左转");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 测试4: 停止
    ESP_LOGI(TAG, "Test 4: Stop");
    controller->ProcessVoiceCommand("停止");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 测试5: 右转
    ESP_LOGI(TAG, "Test 5: Turn right");
    controller->ProcessVoiceCommand("右转");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 测试6: 停止
    ESP_LOGI(TAG, "Test 6: Stop");
    controller->ProcessVoiceCommand("停止");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 测试7: 后退
    ESP_LOGI(TAG, "Test 7: Backward movement");
    controller->ProcessVoiceCommand("后退");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // 测试8: 停止
    ESP_LOGI(TAG, "Test 8: Stop");
    controller->ProcessVoiceCommand("停止");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 测试9: 开灯
    ESP_LOGI(TAG, "Test 9: Turn on light");
    controller->ProcessVoiceCommand("开灯");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 测试10: 关灯
    ESP_LOGI(TAG, "Test 10: Turn off light");
    controller->ProcessVoiceCommand("关灯");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 测试11: 直接速度控制
    ESP_LOGI(TAG, "Test 11: Direct velocity control");
    controller->SendMovementCommand(MovementCommand::FORWARD, 0.3f, 0.0f, 2.0f);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    ESP_LOGI(TAG, "Robot control test completed!");
    
    // 删除任务
    vTaskDelete(NULL);
}

// 启动测试
void StartRobotTest() {
    static RobotMovementController controller(UART_NUM_0, 43, 44, 115200);
    
    if (controller.Initialize()) {
        ESP_LOGI(TAG, "Robot controller initialized successfully");
        
        // 创建测试任务
        xTaskCreate(RobotTestTask, "robot_test", 4096, &controller, 5, NULL);
    } else {
        ESP_LOGE(TAG, "Failed to initialize robot controller");
    }
} 