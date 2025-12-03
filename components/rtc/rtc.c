/**
 * @file rtc.c
 * @brief RTC 驱动占位实现，负责从 GNSS 时间同步到外设。
 */

#include "rtc.h"
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "rtc";
static rtc_config_t s_rtc_cfg;

esp_err_t rtc_init(const rtc_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    s_rtc_cfg = *config;

    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = config->sda_pin,
        .scl_io_num = config->scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    ESP_LOGI(TAG, "初始化 RTC I2C%d (SDA=%d, SCL=%d)", config->i2c_port, config->sda_pin, config->scl_pin);
    ESP_RETURN_ON_ERROR(i2c_param_config(config->i2c_port, &cfg), TAG, "配置 RTC I2C 失败");
    return i2c_driver_install(config->i2c_port, cfg.mode, 0, 0, 0);
}

esp_err_t rtc_sync_from_gnss(uint64_t timestamp_ms)
{
    ESP_LOGI(TAG, "同步 RTC 到 GNSS 时间戳 %llu", (unsigned long long)timestamp_ms);
    // 写入 RTC 芯片寄存器的逻辑留待具体实现。
    return ESP_OK;
}
