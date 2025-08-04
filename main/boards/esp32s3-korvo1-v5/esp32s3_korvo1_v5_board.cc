#include "wifi_board.h"
#include "codecs/box_audio_codec.h"
#include "codecs/es8311_audio_codec.h"
#include "application.h"
#include "button.h"
#include "multi_button.h"
#include "config.h"
#include "i2c_device.h"
#include "led/ws2812_led.h"

// ADC校准函数声明
void start_adc_calibration();


#include <esp_log.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <wifi_station.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "esp32s3_korvo1_v5"

class Esp32S3Korvo1V5Board : public WifiBoard {
private:
    Button boot_button_;
    MultiButton* multi_button_;
    i2c_master_bus_handle_t i2c_bus_;
    WS2812Led* ws2812_led_;

    void InitializeI2c() {
        // 初始化I2C外设 - 用于音频编解码器
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)1,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
        
        ESP_LOGI(TAG, "I2C initialized for audio codecs");
        ESP_LOGI(TAG, "I2C pins: SDA=%d, SCL=%d", AUDIO_CODEC_I2C_SDA_PIN, AUDIO_CODEC_I2C_SCL_PIN);
    }

    void I2cDetect() {
        uint8_t address;
        ESP_LOGI(TAG, "I2C device detection:");
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
        for (int i = 0; i < 128; i += 16) {
            printf("%02x: ", i);
            for (int j = 0; j < 16; j++) {
                fflush(stdout);
                address = i + j;
                esp_err_t ret = i2c_master_probe(i2c_bus_, address, pdMS_TO_TICKS(200));
                if (ret == ESP_OK) {
                    printf("%02x ", address);
                    // 检查是否是音频编解码器
                    if (address == AUDIO_CODEC_ES8311_ADDR) {
                        ESP_LOGI(TAG, "Found ES8311 audio codec at 0x%02x", address);
                    } else if (address == AUDIO_CODEC_ES7210_ADDR) {
                        ESP_LOGI(TAG, "Found ES7210 microphone array at 0x%02x", address);
                    }
                } else if (ret == ESP_ERR_TIMEOUT) {
                    printf("UU ");
                } else {
                    printf("-- ");
                }
            }
            printf("\r\n");
        }
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });

#if CONFIG_USE_DEVICE_AEC
        boot_button_.OnDoubleClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateIdle) {
                app.SetAecMode(app.GetAecMode() == kAecOff ? kAecOnDeviceSide : kAecOff);
            }
        });
#endif

        // 初始化多按键
        InitializeMultiButtons();
    }

    void InitializeMultiButtons() {
        // 定义6个按键的ADC值范围（根据实际测试结果调整）
        std::vector<MultiButton::ButtonConfig> button_configs = {
            {1, 300, 500, "Button1"},      // 按键1: ADC ~400 (Toggle chat)
            {2, 2700, 2900, "Button2"},    // 按键2: ADC ~2794 (Volume up)
            {3, 2000, 2400, "Button3"},    // 按键3: ADC ~1889 (Volume down)
            {4, 800, 1000, "Button4"},     // 按键4: ADC ~900 (Toggle LED)
            {5, 1200, 1400, "Button5"},    // 按键5: ADC ~2279 (Reset WiFi)
            {6, 1800, 1900, "Button6"}     // 按键6: ADC ~1250 (Toggle AEC)
        };

        multi_button_ = new MultiButton(MULTI_BUTTON_GPIO, button_configs);

        // 设置按键回调函数
        multi_button_->OnButtonPress(0, [this]() {
            ESP_LOGI(TAG, "Button 1 pressed - Volume up");
            // 在任务中执行音量调节
            xTaskCreate([](void* param) {
                auto* board = static_cast<Esp32S3Korvo1V5Board*>(param);
                auto* codec = board->GetAudioCodec();
                if (codec) {
                    int current_vol = codec->output_volume();
                    codec->SetOutputVolume(std::min(100, current_vol + 10));
                }
                vTaskDelete(nullptr);
            }, "volume_up", 4096, this, 5, nullptr);
        });

        multi_button_->OnButtonPress(1, [this]() {
            ESP_LOGI(TAG, "Button 2 pressed");
            // 在任务中执行音量调节
            // xTaskCreate([](void* param) {
            //     auto* board = static_cast<Esp32S3Korvo1V5Board*>(param);
            //     auto* codec = board->GetAudioCodec();
            //     if (codec) {
            //         int current_vol = codec->output_volume();
            //         codec->SetOutputVolume(std::min(100, current_vol + 10));
            //     }
            //     vTaskDelete(nullptr);
            // }, "volume_up", 4096, this, 5, nullptr);
        });

        multi_button_->OnButtonPress(2, [this]() {
            ESP_LOGI(TAG, "Button 3 pressed");
            // 在任务中执行LED控制
            // xTaskCreate([](void* param) {
            //     auto* board = static_cast<Esp32S3Korvo1V5Board*>(param);
            //     auto* led = board->GetLed();
            //     if (led) {
            //         auto* ws2812_led = static_cast<WS2812Led*>(led);
            //         if (ws2812_led->GetCurrentEffectState() == LedEffectState::BREATHING) {
            //             ws2812_led->StopBreathingEffect();
            //         } else {
            //             ws2812_led->StartBreathingEffect();
            //         }
            //     }
            //     vTaskDelete(nullptr);
            // }, "led_toggle", 4096, this, 5, nullptr);
        });

        multi_button_->OnButtonPress(3, [this]() {
            ESP_LOGI(TAG, "Button 4 pressed - volume down");
            // 在任务中执行音量调节
            xTaskCreate([](void* param) {
                auto* board = static_cast<Esp32S3Korvo1V5Board*>(param);
                auto* codec = board->GetAudioCodec();
                if (codec) {
                    int current_vol = codec->output_volume();
                    codec->SetOutputVolume(std::max(0, current_vol - 10));
                }
                vTaskDelete(nullptr);
            }, "volume_down", 4096, this, 5, nullptr);
         
        });

        multi_button_->OnButtonPress(4, [this]() {
            ESP_LOGI(TAG, "Button 5 pressed");
            // // 在任务中执行WiFi重置，避免在定时器回调中直接调用
            // xTaskCreate([](void* param) {
            //     auto* board = static_cast<Esp32S3Korvo1V5Board*>(param);
            //     board->ResetWifiConfiguration();
            //     vTaskDelete(nullptr);
            // }, "wifi_reset", 4096, this, 5, nullptr);
        });

        multi_button_->OnButtonPress(5, [this]() {
            ESP_LOGI(TAG, "Button 6 pressed - Toggle chat state");
            // 在任务中执行应用状态切换
            xTaskCreate([](void* param) {
                auto& app = Application::GetInstance();
                app.ToggleChatState();
                vTaskDelete(nullptr);
            }, "toggle_chat", 4096, nullptr, 5, nullptr);
        });

        // 长按功能
        multi_button_->OnButtonLongPress(0, [this]() {
            ESP_LOGI(TAG, "Button 1 long pressed");
            // 在任务中执行重启
            // xTaskCreate([](void* param) {
            //     vTaskDelay(pdMS_TO_TICKS(100)); // 短暂延迟确保日志输出
            //     esp_restart();
            // }, "force_reset", 4096, nullptr, 5, nullptr);
        });

        multi_button_->OnButtonLongPress(4, [this]() {
            ESP_LOGI(TAG, "Button 5 long pressed");
            // 这里可以添加恢复出厂设置的逻辑
        });
    }

    void InitializeWS2812() {
        ws2812_led_ = new WS2812Led(WS2812_LED_GPIO, WS2812_LED_COUNT);
        ESP_LOGI(TAG, "WS2812 LED initialized: GPIO=%d, LED count=%d", WS2812_LED_GPIO, WS2812_LED_COUNT);
    }

public:
    Esp32S3Korvo1V5Board() : boot_button_(BOOT_BUTTON_GPIO), multi_button_(nullptr), ws2812_led_(nullptr) {
        ESP_LOGI(TAG, "Initializing ESP32-S3-KORVO1-V5 Board");
        ESP_LOGI(TAG, "Hardware features: 3-array microphone, ES8311+ES7210 codecs, WS2812 LEDs, 6-button matrix, no display, no camera");
        
        InitializeI2c();
        I2cDetect();
        InitializeButtons();
        InitializeWS2812();
        
        // 启动ADC校准任务（用于调试和校准6个按键的ADC值）
        // start_adc_calibration();
        
        ESP_LOGI(TAG, "KORVO1-V5 board initialization completed");
    }

    ~Esp32S3Korvo1V5Board() {
        if (ws2812_led_) {
            delete ws2812_led_;
            ws2812_led_ = nullptr;
        }
        if (multi_button_) {
            delete multi_button_;
            multi_button_ = nullptr;
        }
    }

    virtual AudioCodec* GetAudioCodec() override {
        // ESP_LOGI(TAG, "Creating audio codec with I2S pins: MCLK=%d, BCLK=%d, WS=%d, DOUT=%d, DIN=%d", 
        //          AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, 
        //          AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
        // ESP_LOGI(TAG, "ES8311 I2S pins: MCLK=%d, BCLK=%d, WS=%d, DOUT=%d", 
        //          AUDIO_ES8311_I2S_GPIO_MCLK, AUDIO_ES8311_I2S_GPIO_BCLK, 
        //          AUDIO_ES8311_I2S_GPIO_WS, AUDIO_ES8311_I2S_GPIO_DOUT);
        // ESP_LOGI(TAG, "Audio codec addresses: ES8311=0x%02x, ES7210=0x%02x, PA_PIN=%d", 
        //          AUDIO_CODEC_ES8311_ADDR, AUDIO_CODEC_ES7210_ADDR, AUDIO_CODEC_PA_PIN);
        
        static BoxAudioCodec audio_codec(
            i2c_bus_, 
            AUDIO_INPUT_SAMPLE_RATE, 
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, 
            AUDIO_I2S_GPIO_BCLK, 
            AUDIO_I2S_GPIO_WS, 
            AUDIO_I2S_GPIO_DOUT, 
            AUDIO_I2S_GPIO_DIN,
            AUDIO_ES8311_I2S_GPIO_MCLK, 
            AUDIO_ES8311_I2S_GPIO_BCLK, 
            AUDIO_ES8311_I2S_GPIO_WS, 
            AUDIO_ES8311_I2S_GPIO_DOUT,
            AUDIO_CODEC_PA_PIN, 
            AUDIO_CODEC_ES8311_ADDR, 
            AUDIO_CODEC_ES7210_ADDR, 
            AUDIO_INPUT_REFERENCE);
        
        ESP_LOGI(TAG, "Audio codec created successfully");
        return &audio_codec;
    }

    virtual Display *GetDisplay() override {
        return nullptr;  // KORVO1 V5 没有显示屏
    }
    
    virtual Camera* GetCamera() override {
        return nullptr;  // KORVO1 V5 没有摄像头
    }
    
    virtual Led* GetLed() override {
        return ws2812_led_;  // 返回WS2812 LED实例
    }
};

DECLARE_BOARD(Esp32S3Korvo1V5Board); 