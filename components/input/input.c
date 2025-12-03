/**
 * @file input.c
 * @brief 输入事件采集与防抖实现。
 */

#include "input.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "input";
static input_config_t s_input_cfg;
static uint32_t s_last_press_tick = 0;

esp_err_t input_init(const input_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    s_input_cfg = *config;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->button_pin) | (1ULL << config->encoder_a_pin) | (1ULL << config->encoder_b_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_LOGI(TAG, "配置输入引脚 BTN=%d, ENC_A=%d, ENC_B=%d", config->button_pin, config->encoder_a_pin, config->encoder_b_pin);
    return gpio_config(&io_conf);
}

static bool debounce_gpio(int pin)
{
    int level = gpio_get_level(pin);
    vTaskDelay(pdMS_TO_TICKS(s_input_cfg.debounce_ms));
    return level == gpio_get_level(pin);
}

esp_err_t input_poll_event(input_event_t *event)
{
    if (!event) {
        return ESP_ERR_INVALID_ARG;
    }
    event->type = INPUT_EVENT_NONE;

    if (debounce_gpio(s_input_cfg.button_pin)) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        uint32_t duration = now - s_last_press_tick;
        s_last_press_tick = now;
        if (duration >= 1000) {
            event->type = INPUT_EVENT_LONG_PRESS;
            event->priority = 2;
        } else {
            event->type = INPUT_EVENT_CLICK;
            event->priority = 1;
        }
        event->timestamp_ms = now;
        return ESP_OK;
    }

    int a = gpio_get_level(s_input_cfg.encoder_a_pin);
    int b = gpio_get_level(s_input_cfg.encoder_b_pin);
    if (a != b) {
        event->type = (a > b) ? INPUT_EVENT_ENCODER_INC : INPUT_EVENT_ENCODER_DEC;
        event->priority = 1;
        event->timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        return ESP_OK;
    }

    return ESP_OK;
}
