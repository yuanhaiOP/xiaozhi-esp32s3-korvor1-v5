#include "box_audio_codec.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <driver/i2s_tdm.h>
#include <driver/i2s_std.h>
#include <algorithm>

// ES8311 寄存器定义
#define ES8311_RESET_REG00       0x00
#define ES8311_CLK_MANAGER_REG01 0x01
#define ES8311_CLK_MANAGER_REG02 0x02
#define ES8311_CLK_MANAGER_REG03 0x03
#define ES8311_CLK_MANAGER_REG04 0x04
#define ES8311_CLK_MANAGER_REG05 0x05
#define ES8311_CLK_MANAGER_REG06 0x06
#define ES8311_CLK_MANAGER_REG07 0x07
#define ES8311_CLK_MANAGER_REG08 0x08
#define ES8311_SDPIN_REG09       0x09
#define ES8311_SDPOUT_REG0A      0x0A
#define ES8311_SYSTEM_REG0B      0x0B
#define ES8311_SYSTEM_REG0C      0x0C
#define ES8311_SYSTEM_REG0D      0x0D
#define ES8311_SYSTEM_REG0E      0x0E
#define ES8311_SYSTEM_REG0F      0x0F
#define ES8311_SYSTEM_REG10      0x10
#define ES8311_SYSTEM_REG11      0x11
#define ES8311_SYSTEM_REG12      0x12
#define ES8311_SYSTEM_REG13      0x13
#define ES8311_SYSTEM_REG14      0x14
#define ES8311_ADC_REG15         0x15
#define ES8311_ADC_REG16         0x16
#define ES8311_ADC_REG17         0x17
#define ES8311_ADC_REG18         0x18
#define ES8311_ADC_REG19         0x19
#define ES8311_ADC_REG1A         0x1A
#define ES8311_ADC_REG1B         0x1B
#define ES8311_ADC_REG1C         0x1C
#define ES8311_DAC_REG31         0x31
#define ES8311_DAC_REG32         0x32
#define ES8311_DAC_REG33         0x33
#define ES8311_DAC_REG34         0x34
#define ES8311_DAC_REG35         0x35
#define ES8311_DAC_REG37         0x37
#define ES8311_GPIO_REG44        0x44
#define ES8311_GP_REG45          0x45
#define ES8311_CHD1_REGFD        0xFD
#define ES8311_CHD2_REGFE        0xFE
#define ES8311_CHVER_REGFF       0xFF

#define TAG "BoxAudioCodec"

BoxAudioCodec::BoxAudioCodec(void* i2c_master_handle, int input_sample_rate, int output_sample_rate,
    gpio_num_t mclk, gpio_num_t bclk, gpio_num_t ws, gpio_num_t dout, gpio_num_t din,
    gpio_num_t es8311_mclk, gpio_num_t es8311_bclk, gpio_num_t es8311_ws, gpio_num_t es8311_dout,
    gpio_num_t pa_pin, uint8_t es8311_addr, uint8_t es7210_addr, bool input_reference) {
    duplex_ = true; // 是否双工
    input_reference_ = input_reference; // 是否使用参考输入，实现回声消除
    input_channels_ = input_reference_ ? 2 : 1; // 输入通道数
    input_sample_rate_ = input_sample_rate;
    output_sample_rate_ = output_sample_rate;
    pa_pin_ = pa_pin;  // 保存功放使能引脚
    ESP_LOGI(TAG, "PA pin configured: %d", pa_pin_);
    
    // 使用两个独立的 I2S 端口
    // ES7210 使用 I2S_NUM_0，ES8311 使用 I2S_NUM_1
    ESP_LOGI(TAG, "Using two independent I2S ports");
    ESP_LOGI(TAG, "ES7210 (I2S_NUM_0): MCLK=%d, WS=%d, BCLK=%d, DIN=%d, DOUT=%d", mclk, ws, bclk, din, dout);
    ESP_LOGI(TAG, "ES8311 (I2S_NUM_1): MCLK=%d, WS=%d, BCLK=%d, DOUT=%d", es8311_mclk, es8311_ws, es8311_bclk, es8311_dout);
    
    // 为 ES7210 创建 I2S_NUM_0 通道
    CreateES7210Channels(mclk, bclk, ws, dout, din);
    
    // 为 ES8311 创建 I2S_NUM_1 通道
    CreateES8311Channels(es8311_mclk, es8311_bclk, es8311_ws, es8311_dout);

    // 为 ES7210 创建数据接口（使用 I2S_NUM_0）
    // ES7210 只需要 RX 通道，因为它是输入设备
    audio_codec_i2s_cfg_t es7210_i2s_cfg = {
        .port = I2S_NUM_0,
        .rx_handle = es7210_rx_handle_,
        .tx_handle = nullptr,  // ES7210 不需要 TX 通道
    };
    const audio_codec_data_if_t* es7210_data_if_ = audio_codec_new_i2s_data(&es7210_i2s_cfg);
    assert(es7210_data_if_ != NULL);

    // 为 ES8311 创建数据接口（使用 I2S_NUM_1）
    // ES8311 只需要 TX 通道，因为它是输出设备
    audio_codec_i2s_cfg_t es8311_i2s_cfg = {
        .port = I2S_NUM_1,
        .rx_handle = nullptr,  // ES8311 不需要 RX 通道
        .tx_handle = es8311_tx_handle_,
    };
    const audio_codec_data_if_t* es8311_data_if_ = audio_codec_new_i2s_data(&es8311_i2s_cfg);
    assert(es8311_data_if_ != NULL);

    // Output
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = (i2c_port_t)1,
        .addr = es8311_addr,
        .bus_handle = i2c_master_handle,
    };
    out_ctrl_if_ = audio_codec_new_i2c_ctrl(&i2c_cfg);
    assert(out_ctrl_if_ != NULL);

    gpio_if_ = audio_codec_new_gpio();
    assert(gpio_if_ != NULL);

    // 配置ES8311（使用专用的I2S引脚）
    es8311_codec_cfg_t es8311_cfg = {};
    es8311_cfg.ctrl_if = out_ctrl_if_;
    es8311_cfg.gpio_if = gpio_if_;
    es8311_cfg.codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC;  // 参考chinese_tts
    es8311_cfg.pa_pin = pa_pin;
    es8311_cfg.pa_reverted = pa_reverted_;  // 使用传入的 PA 反转参数
    ESP_LOGI(TAG, "ES8311 config: pa_pin=%d, pa_reverted=%s", pa_pin, es8311_cfg.pa_reverted ? "true" : "false");
    es8311_cfg.use_mclk = (es8311_mclk != GPIO_NUM_NC);  // 如果 MCLK 未连接，则不使用
    es8311_cfg.hw_gain.pa_voltage = 5.0;
    es8311_cfg.hw_gain.codec_dac_voltage = 3.3;
    
    // ES8311 的 I2S 引脚配置通过 gpio_if 处理，不需要在这里设置
    
    ESP_LOGI(TAG, "ES8311 I2S pins: MCLK=%d, BCLK=%d, WS=%d, DOUT=%d", 
             es8311_mclk, es8311_bclk, es8311_ws, es8311_dout);
    ESP_LOGI(TAG, "ES8311 use_mclk: %s", es8311_cfg.use_mclk ? "true" : "false");
    out_codec_if_ = es8311_codec_new(&es8311_cfg);
    if (out_codec_if_ == NULL) {
        ESP_LOGE(TAG, "Failed to create ES8311 codec interface");
        assert(false);
    } else {
        ESP_LOGI(TAG, "ES8311 codec interface created successfully");
    }

    // 为 ES8311 创建设备配置（使用 I2S_NUM_1）
    esp_codec_dev_cfg_t es8311_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = out_codec_if_,
        .data_if = es8311_data_if_,
    };
    output_dev_ = esp_codec_dev_new(&es8311_dev_cfg);
    if (output_dev_ == NULL) {
        ESP_LOGE(TAG, "Failed to create ES8311 output device");
        assert(false);
    } else {
        ESP_LOGI(TAG, "ES8311 output device created successfully");
    }

    // Input - ES7210
    i2c_cfg.addr = es7210_addr;
    in_ctrl_if_ = audio_codec_new_i2c_ctrl(&i2c_cfg);
    assert(in_ctrl_if_ != NULL);

    es7210_codec_cfg_t es7210_cfg = {};
    es7210_cfg.ctrl_if = in_ctrl_if_;
    // KORVO1-V5只有3个麦克风，只启用MIC1-MIC3，但TDM/I2S必须配置为4通道
    es7210_cfg.mic_selected = ES7120_SEL_MIC1 | ES7120_SEL_MIC2 | ES7120_SEL_MIC3;
    ESP_LOGI(TAG, "Creating ES7210 codec with mic_selected=0x%x", es7210_cfg.mic_selected);
    in_codec_if_ = es7210_codec_new(&es7210_cfg);
    assert(in_codec_if_ != NULL);

    // 为 ES7210 创建设备配置（使用 I2S_NUM_0）
    esp_codec_dev_cfg_t es7210_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = in_codec_if_,
        .data_if = es7210_data_if_,
    };
    input_dev_ = esp_codec_dev_new(&es7210_dev_cfg);
    assert(input_dev_ != NULL);

    ESP_LOGI(TAG, "BoxAudioDevice initialized");
}

BoxAudioCodec::~BoxAudioCodec() {
    ESP_ERROR_CHECK(esp_codec_dev_close(output_dev_));
    esp_codec_dev_delete(output_dev_);
    ESP_ERROR_CHECK(esp_codec_dev_close(input_dev_));
    esp_codec_dev_delete(input_dev_);

    audio_codec_delete_codec_if(in_codec_if_);
    audio_codec_delete_ctrl_if(in_ctrl_if_);
    audio_codec_delete_codec_if(out_codec_if_);
    audio_codec_delete_ctrl_if(out_ctrl_if_);
    audio_codec_delete_gpio_if(gpio_if_);
    
    // 清理 I2S 通道
    if (es7210_rx_handle_) {
        i2s_del_channel(es7210_rx_handle_);
    }
    if (es8311_tx_handle_) {
        i2s_del_channel(es8311_tx_handle_);
    }
}

void BoxAudioCodec::CreateDuplexChannels(gpio_num_t mclk, gpio_num_t bclk, gpio_num_t ws, gpio_num_t dout, gpio_num_t din) {
    assert(input_sample_rate_ == output_sample_rate_);

    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_0,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = AUDIO_CODEC_DMA_DESC_NUM,
        .dma_frame_num = AUDIO_CODEC_DMA_FRAME_NUM,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
        .intr_priority = 0,
    };
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle_, &rx_handle_));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = (uint32_t)output_sample_rate_,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = I2S_STD_SLOT_BOTH,
            .ws_width = I2S_DATA_BIT_WIDTH_16BIT,
            .ws_pol = false,
            .bit_shift = true,
            .left_align = true,
            .big_endian = false,
            .bit_order_lsb = false
        },
        .gpio_cfg = {
            .mclk = mclk,
            .bclk = bclk,
            .ws = ws,
            .dout = dout,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };

    i2s_tdm_config_t tdm_cfg = {
        .clk_cfg = {
            .sample_rate_hz = (uint32_t)input_sample_rate_,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
            .bclk_div = 8,
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            // 必须配置为4通道，slot_mask选4个slot
            .slot_mask = i2s_tdm_slot_mask_t(I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3),
            .ws_width = I2S_TDM_AUTO_WS_WIDTH,
            .ws_pol = false,
            .bit_shift = true,
            .left_align = false,
            .big_endian = false,
            .bit_order_lsb = false,
            .skip_mask = false,
            .total_slot = 4  // 明确指定总槽位数为4
        },
        .gpio_cfg = {
            .mclk = mclk,
            .bclk = bclk,
            .ws = ws,
            .dout = I2S_GPIO_UNUSED,
            .din = din,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(rx_handle_, &tdm_cfg));
    ESP_LOGI(TAG, "Duplex channels created");
}

void BoxAudioCodec::SetOutputVolume(int volume) {
    ESP_ERROR_CHECK(esp_codec_dev_set_out_vol(output_dev_, volume));
    AudioCodec::SetOutputVolume(volume);
}

void BoxAudioCodec::EnableInput(bool enable) {
    if (enable == input_enabled_) {
        return;
    }
    if (enable) {
        esp_codec_dev_sample_info_t fs = {
            .bits_per_sample = 16,
            .channel = 4,  // 必须为4通道，后续只用前三路数据
            .channel_mask = ESP_CODEC_DEV_MAKE_CHANNEL_MASK(0),
            .sample_rate = (uint32_t)output_sample_rate_,
            .mclk_multiple = 0,
        };
        if (input_reference_) {
            fs.channel_mask |= ESP_CODEC_DEV_MAKE_CHANNEL_MASK(1);
        }
        ESP_ERROR_CHECK(esp_codec_dev_open(input_dev_, &fs));
        ESP_ERROR_CHECK(esp_codec_dev_set_in_channel_gain(input_dev_, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(0), AUDIO_CODEC_DEFAULT_MIC_GAIN));
        ESP_LOGI(TAG, "Input device opened with %d channels", fs.channel);
    } else {
        ESP_ERROR_CHECK(esp_codec_dev_close(input_dev_));
    }
    AudioCodec::EnableInput(enable);
}

void BoxAudioCodec::EnableOutput(bool enable) {
    if (enable == output_enabled_) {
        return;
    }
    if (enable) {
        ESP_LOGI(TAG, "Enabling ES8311 output...");
        
        // 检查 PA 引脚配置
        ESP_LOGI(TAG, "PA pin: %d, PA reverted: %s", 
                 pa_pin_, pa_reverted_ ? "true" : "false");
        
                       // 检查 PA 引脚状态（只监控，不控制）
               if (pa_pin_ != GPIO_NUM_NC) {
                   int pa_level = gpio_get_level(pa_pin_);
                   ESP_LOGI(TAG, "PA pin (GPIO %d) level: %d", pa_pin_, pa_level);
                   
                   // 检查 PA 引脚状态
                   if (pa_level == 0) {
                       ESP_LOGW(TAG, "PA pin is LOW - ES8311 driver may not have enabled it yet");
                   } else {
                       ESP_LOGI(TAG, "PA pin is HIGH - ES8311 driver has enabled amplifier");
                   }
               }
        
        // Play 16bit 1 channel
        esp_codec_dev_sample_info_t fs = {
            .bits_per_sample = 16,
            .channel = 1,
            .channel_mask = 0,
            .sample_rate = (uint32_t)output_sample_rate_,
            .mclk_multiple = 0,
        };
        ESP_LOGI(TAG, "Audio format: %d bits, %d channels, %lu Hz", 
                 fs.bits_per_sample, fs.channel, (unsigned long)fs.sample_rate);
        
        // 检查音频格式是否与 ES8311 兼容
        if (fs.bits_per_sample != 16) {
            ESP_LOGW(TAG, "Warning: ES8311 expects 16-bit samples, got %d-bit", fs.bits_per_sample);
        }
        if (fs.channel != 1) {
            ESP_LOGW(TAG, "Warning: ES8311 expects 1 channel, got %d channels", fs.channel);
        }
        if (fs.sample_rate != 24000) {
            ESP_LOGW(TAG, "Warning: ES8311 expects 24kHz, got %lu Hz", (unsigned long)fs.sample_rate);
        }
        
        ESP_ERROR_CHECK(esp_codec_dev_open(output_dev_, &fs));
        ESP_ERROR_CHECK(esp_codec_dev_set_out_vol(output_dev_, output_volume_));
        
        ESP_LOGI(TAG, "Output device opened with volume %d", output_volume_);
        
        // 设置最大音量
        ESP_LOGI(TAG, "Setting ES8311 volume to maximum...");
        esp_codec_dev_set_out_vol(output_dev_, 100);
        
        // 检查 ES8311 寄存器状态（仅用于调试）
        CheckES8311Registers();
        
        ESP_LOGI(TAG, "ES8311 output enabled successfully");
        
        output_enabled_ = true;
    } else {
        ESP_LOGI(TAG, "Disabling ES8311 output...");
        
        ESP_ERROR_CHECK(esp_codec_dev_close(output_dev_));
        
        ESP_LOGI(TAG, "Output device closed");
        output_enabled_ = false;
    }
}

int BoxAudioCodec::Read(int16_t* dest, int samples) {
    if (input_enabled_) {
        esp_err_t ret = esp_codec_dev_read(input_dev_, (void*)dest, samples * sizeof(int16_t));
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "esp_codec_dev_read failed: %s", esp_err_to_name(ret));
            // 返回0表示没有读取到数据
            return 0;
        }
    }
    return samples;
}

int BoxAudioCodec::Write(const int16_t* data, int samples) {
    if (output_enabled_) {
        static int total_samples_written = 0;
        total_samples_written += samples;
        
        // 详细的音频数据检查
        bool has_audio_data = false;
        int16_t max_sample = 0;
        int16_t min_sample = 0;
        int non_zero_count = 0;
        
        for (int i = 0; i < samples && i < 100; i++) {
            if (data[i] != 0) {
                has_audio_data = true;
                non_zero_count++;
                max_sample = (data[i] > max_sample) ? data[i] : max_sample;
                min_sample = (data[i] < min_sample) ? data[i] : min_sample;
            }
        }
        
        ESP_LOGI(TAG, "Audio data: has_audio=%s, non_zero=%d/%d, range=[%d, %d]", 
                 has_audio_data ? "YES" : "NO", non_zero_count, samples, min_sample, max_sample);
        
        // 检查I2S写入结果
        esp_err_t ret = esp_codec_dev_write(output_dev_, (void*)data, samples * sizeof(int16_t));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "esp_codec_dev_write failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGD(TAG, "I2S write successful: %d bytes", samples * sizeof(int16_t));
            
            // 检查音频数据是否真的被写入
            static int total_bytes_written = 0;
            total_bytes_written += samples * sizeof(int16_t);
            
            // 每10000个样本检查一次
            static int sample_count = 0;
            sample_count += samples;
            if (sample_count >= 10000) {
                ESP_LOGI(TAG, "Audio data flow: %d bytes written, %d samples processed", 
                         total_bytes_written, sample_count);
                sample_count = 0;
            }
            
                               // 每1000次写入检查一次ES8311状态
                   static int write_count = 0;
                   write_count++;
                   if (write_count % 1000 == 0) {
                       ESP_LOGI(TAG, "ES8311 write count: %d, total samples: %d", write_count, total_samples_written);
                       
                       // 检查 PA 引脚状态
                       if (pa_pin_ != GPIO_NUM_NC) {
                           int pa_level = gpio_get_level(pa_pin_);
                           if (pa_level == 0) {
                               ESP_LOGW(TAG, "PA pin level during playback: %d (LOW - might be issue)", pa_level);
                           } else {
                               ESP_LOGI(TAG, "PA pin level during playback: %d (HIGH - normal)", pa_level);
                           }
                       }
                       
                       // 定期检查 ES8311 寄存器状态
                       if (write_count % 5000 == 0) {
                           CheckES8311Registers();
                       }
                   }
        }
    } else {
        ESP_LOGW(TAG, "Output not enabled, cannot write audio data");
    }
    return samples;
}

void BoxAudioCodec::CheckES8311Registers() {
    ESP_LOGI(TAG, "=== ES8311 Register Check ===");
    
    // 暂时简化，只检查控制接口是否可用
    if (out_ctrl_if_ && out_ctrl_if_->read_reg) {
        ESP_LOGI(TAG, "ES8311 control interface is available");
        
        // 尝试读取芯片 ID
        uint8_t chip_id1 = 0, chip_id2 = 0, version = 0;
        esp_err_t ret1 = out_ctrl_if_->read_reg(out_ctrl_if_, ES8311_CHD1_REGFD, 1, &chip_id1, 1);
        esp_err_t ret2 = out_ctrl_if_->read_reg(out_ctrl_if_, ES8311_CHD2_REGFE, 1, &chip_id2, 1);
        esp_err_t ret3 = out_ctrl_if_->read_reg(out_ctrl_if_, ES8311_CHVER_REGFF, 1, &version, 1);
        
        if (ret1 == ESP_OK && ret2 == ESP_OK && ret3 == ESP_OK) {
            ESP_LOGI(TAG, "Chip ID: 0x%02x 0x%02x, Version: 0x%02x", chip_id1, chip_id2, version);
        } else {
            ESP_LOGE(TAG, "Failed to read chip ID: ret1=%d, ret2=%d, ret3=%d", ret1, ret2, ret3);
        }
        
                       // 尝试读取 DAC 音量寄存器
               uint8_t dac_volume = 0;
               esp_err_t ret_vol = out_ctrl_if_->read_reg(out_ctrl_if_, ES8311_DAC_REG32, 1, &dac_volume, 1);
               if (ret_vol == ESP_OK) {
                   ESP_LOGI(TAG, "DAC Volume Register: 0x%02x", dac_volume);
               } else {
                   ESP_LOGE(TAG, "Failed to read DAC volume register: %d", ret_vol);
               }
               
               // 检查 DAC 输出使能寄存器
               uint8_t dac_output_enable = 0;
               esp_err_t ret_out = out_ctrl_if_->read_reg(out_ctrl_if_, ES8311_DAC_REG31, 1, &dac_output_enable, 1);
               if (ret_out == ESP_OK) {
                   ESP_LOGI(TAG, "DAC Output Enable Register: 0x%02x", dac_output_enable);
               } else {
                   ESP_LOGE(TAG, "Failed to read DAC output enable register: %d", ret_out);
               }
               
               // 检查系统控制寄存器
               uint8_t system_ctrl = 0;
               esp_err_t ret_sys = out_ctrl_if_->read_reg(out_ctrl_if_, ES8311_SYSTEM_REG0D, 1, &system_ctrl, 1);
               if (ret_sys == ESP_OK) {
                   ESP_LOGI(TAG, "System Control Register: 0x%02x", system_ctrl);
               } else {
                   ESP_LOGE(TAG, "Failed to read system control register: %d", ret_sys);
               }
        
    } else {
        ESP_LOGE(TAG, "Cannot read ES8311 registers - control interface not available");
    }
    
    ESP_LOGI(TAG, "=== End ES8311 Register Check ===");
}

void BoxAudioCodec::CreateES7210Channels(gpio_num_t mclk, gpio_num_t bclk, gpio_num_t ws, gpio_num_t dout, gpio_num_t din) {
    ESP_LOGI(TAG, "Creating ES7210 channels on I2S_NUM_0");
    
    // 创建 ES7210 的 I2S 通道（I2S_NUM_0）
    // ES7210 只需要 RX 通道，因为它是输入设备
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_0,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = AUDIO_CODEC_DMA_DESC_NUM,
        .dma_frame_num = AUDIO_CODEC_DMA_FRAME_NUM,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
        .intr_priority = 0,
    };
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, nullptr, &es7210_rx_handle_));  // 只创建 RX 通道

    // ES7210 使用 TDM 模式进行输入
    i2s_tdm_config_t tdm_cfg = {
        .clk_cfg = {
            .sample_rate_hz = (uint32_t)input_sample_rate_,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
            .bclk_div = 8,
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = i2s_tdm_slot_mask_t(I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3),
            .ws_width = I2S_TDM_AUTO_WS_WIDTH,
            .ws_pol = false,
            .bit_shift = true,
            .left_align = false,
            .big_endian = false,
            .bit_order_lsb = false,
            .skip_mask = false,
            .total_slot = 4
        },
        .gpio_cfg = {
            .mclk = mclk,
            .bclk = bclk,
            .ws = ws,
            .dout = I2S_GPIO_UNUSED,
            .din = din,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };

    ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(es7210_rx_handle_, &tdm_cfg));
    
    // 启动 ES7210 的 RX 通道
    ESP_ERROR_CHECK(i2s_channel_enable(es7210_rx_handle_));
    ESP_LOGI(TAG, "ES7210 channels created and enabled successfully");
}

void BoxAudioCodec::CreateES8311Channels(gpio_num_t mclk, gpio_num_t bclk, gpio_num_t ws, gpio_num_t dout) {
    ESP_LOGI(TAG, "Creating ES8311 channels on I2S_NUM_1");
    
    // 创建 ES8311 的 I2S 通道（I2S_NUM_1）
    // ES8311 只需要 TX 通道，因为它是输出设备
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_1,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = AUDIO_CODEC_DMA_DESC_NUM,
        .dma_frame_num = AUDIO_CODEC_DMA_FRAME_NUM,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
        .intr_priority = 0,
    };
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &es8311_tx_handle_, nullptr));  // 只创建 TX 通道

    // ES8311 使用标准 I2S 模式进行输出
    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = (uint32_t)output_sample_rate_,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .ext_clk_freq_hz = 0,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = I2S_STD_SLOT_BOTH,
            .ws_width = I2S_DATA_BIT_WIDTH_16BIT,
            .ws_pol = false,
            .bit_shift = true,
            .left_align = true,
            .big_endian = false,
            .bit_order_lsb = false
        },
        .gpio_cfg = {
            .mclk = (mclk == GPIO_NUM_NC) ? I2S_GPIO_UNUSED : mclk,  // 如果 MCLK 未连接，则不使用
            .bclk = bclk,
            .ws = ws,
            .dout = dout,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(es8311_tx_handle_, &std_cfg));
    
    // 启动 ES8311 的 TX 通道
    ESP_ERROR_CHECK(i2s_channel_enable(es8311_tx_handle_));
    ESP_LOGI(TAG, "ES8311 channels created and enabled successfully");
}
