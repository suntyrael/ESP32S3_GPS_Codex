/**
 * @file log.c
 * @brief 基础日志输出实现。
 */

#include "log.h"
#include "esp_log.h"

static const char *TAG = "app_log";
static uint32_t s_heartbeat_counter = 0;

esp_err_t log_heartbeat(void)
{
    s_heartbeat_counter++;
    ESP_LOGI(TAG, "心跳 #%u", s_heartbeat_counter);
    return ESP_OK;
}

esp_err_t log_event(const char *message, uint32_t timestamp_ms)
{
    if (!message) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGI(TAG, "[%u] %s", timestamp_ms, message);
    return ESP_OK;
}
