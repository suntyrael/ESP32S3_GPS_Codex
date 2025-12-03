/**
 * @file power.c
 * @brief 电源管理占位实现。
 */

#include "power.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

static const char *TAG = "power";
static power_config_t s_power_cfg;

esp_err_t power_init(const power_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    s_power_cfg = *config;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->sdio_power_pin) | (1ULL << config->sensor_power_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "配置供电 GPIO 失败");
    gpio_set_level(config->sdio_power_pin, 1);
    gpio_set_level(config->sensor_power_pin, 1);

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "配置背光 PWM 失败");

    ledc_channel_config_t channel = {
        .gpio_num = config->backlight_pwm_pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 128,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), TAG, "配置背光通道失败");

    ESP_LOGI(TAG, "电源管理初始化完成");
    return ESP_OK;
}

esp_err_t power_set_backlight(uint8_t duty_percent)
{
    uint32_t duty = (duty_percent * 255) / 100;
    ESP_LOGD(TAG, "设置背光占空比 %u%%", duty_percent);
    return ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty, 0);
}
