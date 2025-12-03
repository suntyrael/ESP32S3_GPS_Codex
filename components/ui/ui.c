/**
 * @file ui.c
 * @brief UI 占位实现，预留 LVGL + LovyanGFX 交互。
 */

#include "ui.h"
#include "esp_log.h"

static const char *TAG = "ui";

esp_err_t ui_init(void)
{
    ESP_LOGI(TAG, "初始化 LVGL 与 LovyanGFX 渲染管线");
    // 真实项目中在此初始化显示驱动、输入设备及 LVGL 任务。
    return ESP_OK;
}

esp_err_t ui_handle_input(int input_event)
{
    ESP_LOGD(TAG, "处理输入事件 %d，切换界面", input_event);
    return ESP_OK;
}

esp_err_t ui_update(bool has_fix)
{
    ESP_LOGD(TAG, "刷新 UI，GNSS 状态=%s", has_fix ? "锁定" : "未锁定");
    return ESP_OK;
}
