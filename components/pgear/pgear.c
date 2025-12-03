/**
 * @file pgear.c
 * @brief P-GEAR 页面逻辑占位实现。
 */

#include "pgear.h"
#include "esp_log.h"

static const char *TAG = "pgear";

esp_err_t pgear_init(void)
{
    ESP_LOGI(TAG, "初始化 P-GEAR 计时资源");
    return ESP_OK;
}

esp_err_t pgear_tick(void)
{
    ESP_LOGD(TAG, "更新 P-GEAR 计时");
    return ESP_OK;
}
