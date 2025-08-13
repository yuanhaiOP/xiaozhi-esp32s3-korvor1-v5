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
    
    // 测试1: 前进 - 验证执行顺序
    ESP_LOGI(TAG, "Test 1: Forward movement - checking execution order");
    ESP_LOGI(TAG, "Before calling ProcessVoiceCommand");
    bool result = controller->ProcessVoiceCommand("前进");
    ESP_LOGI(TAG, "After calling ProcessVoiceCommand, result: %s", result ? "true" : "false");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 测试2: 停止
    ESP_LOGI(TAG, "Test 2: Stop");
    controller->ProcessVoiceCommand("停止");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 测试3: 左转 - 验证执行顺序
    ESP_LOGI(TAG, "Test 3: Turn left - checking execution order");
    ESP_LOGI(TAG, "Before calling ProcessVoiceCommand");
    result = controller->ProcessVoiceCommand("左转");
    ESP_LOGI(TAG, "After calling ProcessVoiceCommand, result: %s", result ? "true" : "false");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 测试4: 停止
    ESP_LOGI(TAG, "Test 4: Stop");
    controller->ProcessVoiceCommand("停止");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 测试5: 转一圈 - 验证执行顺序
    ESP_LOGI(TAG, "Test 5: Turn one circle - checking execution order");
    ESP_LOGI(TAG, "Before calling ProcessVoiceCommand");
    result = controller->ProcessVoiceCommand("转一圈");
    ESP_LOGI(TAG, "After calling ProcessVoiceCommand, result: %s", result ? "true" : "false");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 测试6: 停止
    ESP_LOGI(TAG, "Test 6: Stop");
    controller->ProcessVoiceCommand("停止");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "Robot control test completed!");
    
    // 删除任务
    vTaskDelete(NULL);
}

// 启动测试
void StartRobotTest(RobotMovementController* controller) {
    xTaskCreate(RobotTestTask, "RobotTest", 4096, controller, 5, NULL);
} 