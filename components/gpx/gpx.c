/**
 * @file gpx.c
 * @brief GPX 文件写入占位实现，基于 SDIO 文件系统接口。
 */

#include "gpx.h"
#include "esp_log.h"

static const char *TAG = "gpx";

esp_err_t gpx_begin(void)
{
    ESP_LOGI(TAG, "创建 GPX 文件并写入头部");
    return ESP_OK;
}

esp_err_t gpx_append_fix(const gnss_fix_t *fix)
{
    if (!fix) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGD(TAG, "写入轨迹点 lat=%f lon=%f", fix->latitude, fix->longitude);
    return ESP_OK;
}

esp_err_t gpx_close(void)
{
    ESP_LOGI(TAG, "关闭 GPX 文件");
    return ESP_OK;
}
