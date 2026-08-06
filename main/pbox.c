/*
 * pbox.c - P-Box 性能测试实现
 * 状态机：READY --短按--> ARMED --启动条件--> RUNNING --达目标--> FINISHED --短按--> READY
 */
#include "pbox.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "pbox";

static SemaphoreHandle_t s_mutex = NULL;
static pbox_status_t s_st = { 0 };

static const float s_targets[] = PBOX_TARGETS_KMH;
#define PBOX_TARGET_CNT  (sizeof(s_targets) / sizeof(s_targets[0]))

esp_err_t pbox_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_st.state = PBOX_READY;
    s_st.target_idx = 0;
    s_st.target_kmh = s_targets[0];
    ESP_LOGI(TAG, "pbox init ok (targets: %d)", PBOX_TARGET_CNT);
    return ESP_OK;
}

void pbox_arm(void)
{
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    switch (s_st.state) {
    case PBOX_READY:
        s_st.state = PBOX_ARMED;
        s_st.can_start = false;
        ESP_LOGI(TAG, "P-Box ARMED (0-%.0f km/h)", s_st.target_kmh);
        break;
    case PBOX_ARMED:
        s_st.state = PBOX_READY;
        ESP_LOGI(TAG, "P-Box READY");
        break;
    case PBOX_RUNNING:
        break;    /* 运行中忽略短按 */
    case PBOX_FINISHED:
        s_st.state = PBOX_READY;
        s_st.elapsed_s = 0;
        s_st.max_speed_kmh = 0;
        ESP_LOGI(TAG, "P-Box RESET");
        break;
    }
    xSemaphoreGive(s_mutex);
}

void pbox_update(float speed_kmh, float acc_x_g)
{
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    switch (s_st.state) {
    case PBOX_ARMED: {
        /* 启动条件：静止且向前加速度超阈值 */
        bool cond = (speed_kmh < PBOX_START_SPEED_KMH) && (acc_x_g > PBOX_ACC_THRESHOLD_G);
        s_st.can_start = cond;
        if (cond) {
            s_st.state = PBOX_RUNNING;
            s_st.t0_us = esp_timer_get_time();
            s_st.elapsed_s = 0;
            s_st.max_speed_kmh = 0;
            ESP_LOGI(TAG, "P-Box START!");
        }
        break;
    }
    case PBOX_RUNNING: {
        uint64_t now = esp_timer_get_time();
        s_st.elapsed_s = (float)(now - s_st.t0_us) / 1000000.0f;
        if (speed_kmh > s_st.max_speed_kmh) {
            s_st.max_speed_kmh = speed_kmh;
        }
        if (speed_kmh >= s_st.target_kmh) {
            s_st.state = PBOX_FINISHED;
            ESP_LOGI(TAG, "P-Box FINISHED %.2fs", (double)s_st.elapsed_s);
        }
        break;
    }
    default:
        break;
    }
    xSemaphoreGive(s_mutex);
}

void pbox_get_status(pbox_status_t *out)
{
    if (out == NULL) {
        return;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *out = s_st;
        xSemaphoreGive(s_mutex);
    }
}
