#include "wifi_board.h"
#include "codecs/box_audio_codec.h"
#include "codecs/es8311_audio_codec.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "i2c_device.h"

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
    i2c_master_bus_handle_t i2c_bus_;

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
    }

public:
    Esp32S3Korvo1V5Board() : boot_button_(BOOT_BUTTON_GPIO) {
        ESP_LOGI(TAG, "Initializing ESP32-S3-KORVO1-V5 Board");
        ESP_LOGI(TAG, "Hardware features: 3-array microphone, ES8311+ES7210 codecs, no display, no camera");
        
        InitializeI2c();
        I2cDetect();
        InitializeButtons();
        ESP_LOGI(TAG, "KORVO1-V5 board initialization completed");
    }

    virtual AudioCodec* GetAudioCodec() override {
        ESP_LOGI(TAG, "Creating audio codec with I2S pins: MCLK=%d, BCLK=%d, WS=%d, DOUT=%d, DIN=%d", 
                 AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, 
                 AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
        ESP_LOGI(TAG, "ES8311 I2S pins: MCLK=%d, BCLK=%d, WS=%d, DOUT=%d", 
                 AUDIO_ES8311_I2S_GPIO_MCLK, AUDIO_ES8311_I2S_GPIO_BCLK, 
                 AUDIO_ES8311_I2S_GPIO_WS, AUDIO_ES8311_I2S_GPIO_DOUT);
        ESP_LOGI(TAG, "Audio codec addresses: ES8311=0x%02x, ES7210=0x%02x, PA_PIN=%d", 
                 AUDIO_CODEC_ES8311_ADDR, AUDIO_CODEC_ES7210_ADDR, AUDIO_CODEC_PA_PIN);
        
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
};

DECLARE_BOARD(Esp32S3Korvo1V5Board); 