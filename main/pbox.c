/*
 * pbox.c - P-Box 性能测试实现
 * 状态机：READY --短按--> ARMED --启动条件--> RUNNING --达目标--> FINISHED --短按--> READY
 */
#include "pbox.h"
#include "config.h"
#include <string.h>
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
    s_st.elapsed_s = 0.0f;
    s_st.max_speed_kmh = 0.0f;
    s_st.t_0_60 = 0.0f;
    s_st.t_0_100 = 0.0f;
    s_st.t_400m = 0.0f;
    s_st.slope_pct = 0.0f;
    s_st.peak_g = 0.0f;
    s_st.distance_m = 0.0f;
    ESP_LOGI(TAG, "pbox init ok (targets: %d)", PBOX_TARGET_CNT);
    return ESP_OK;
}

void pbox_arm(void)
{
    if (s_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    /* 短按随时将状态复位为就绪，清零计时器与峰值 */
    s_st.state = PBOX_READY;
    s_st.elapsed_s = 0.0f;
    s_st.max_speed_kmh = 0.0f;
    s_st.peak_g = 0.0f;
    ESP_LOGI(TAG, "P-Box RESET to READY");
    xSemaphoreGive(s_mutex);
}

void pbox_update(float speed_kmh, float acc_x_g)
{
    if (s_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }

    /* 实时追踪真实线性加速度峰值 */
    float abs_g = fabsf(acc_x_g);
    if (abs_g > s_st.peak_g) {
        s_st.peak_g = abs_g;
    }
    switch (s_st.state) {
    case PBOX_READY:
    case PBOX_ARMED: {
        /* 真实起步触发条件：
         * 1. 车辆必须处于静止或低速 (< 3.0 km/h)
         * 2. 踩油门产生向前推力 (acc_x_g > 0.25g 且车速开始抬升) 或 GPS 速度离开零点 (speed_kmh >= 1.5 km/h)
         */
        bool launch_by_spd = (speed_kmh >= 1.5f && speed_kmh < 15.0f);
        bool launch_by_acc = (acc_x_g > 0.25f && speed_kmh >= 0.8f);

        if (launch_by_spd || launch_by_acc) {
            s_st.state = PBOX_RUNNING;
            s_st.t0_us = esp_timer_get_time();
            s_st.elapsed_s = 0.0f;
            s_st.max_speed_kmh = speed_kmh;
            s_st.t_0_60 = 0.0f;
            s_st.t_0_100 = 0.0f;
            s_st.t_400m = 0.0f;
            s_st.slope_pct = 0.0f;
            s_st.distance_m = 0.0f;
            s_st.peak_g = (acc_x_g > 0.0f) ? acc_x_g : 0.0f;
            ESP_LOGI(TAG, "P-Box LAUNCH! spd=%.1f acc=%.2fg", (double)speed_kmh, (double)acc_x_g);
        }
        break;
    }
    case PBOX_RUNNING: {
        uint64_t now = esp_timer_get_time();
        s_st.elapsed_s = (float)(now - s_st.t0_us) / 1000000.0f;

        /* 防误触发保护：起步后 2.5 秒内车速依然 < 2.0 km/h（未真正移动），自动复位回 READY */
        if (s_st.elapsed_s > 2.5f && speed_kmh < 2.0f) {
            s_st.state = PBOX_READY;
            s_st.elapsed_s = 0.0f;
            s_st.max_speed_kmh = 0.0f;
            ESP_LOGI(TAG, "P-Box False Launch Timed Out -> Auto Reset");
            break;
        }

        if (acc_x_g > s_st.peak_g) {
            s_st.peak_g = acc_x_g;
        }
        if (speed_kmh > s_st.max_speed_kmh) {
            s_st.max_speed_kmh = speed_kmh;
        }

        /* 0-60 km/h 记录 */
        if (speed_kmh >= 60.0f && s_st.t_0_60 <= 0.0f) {
            s_st.t_0_60 = s_st.elapsed_s;
            ESP_LOGI(TAG, "0-60 km/h: %.2fs", (double)s_st.t_0_60);
        }

        /* 0-100 km/h 记录 */
        if (speed_kmh >= 100.0f && s_st.t_0_100 <= 0.0f) {
            s_st.t_0_100 = s_st.elapsed_s;
            ESP_LOGI(TAG, "0-100 km/h: %.2fs", (double)s_st.t_0_100);
        }

        /* 达成目标速度完成测试 */
        if (speed_kmh >= s_st.target_kmh) {
            s_st.state = PBOX_FINISHED;
            ESP_LOGI(TAG, "P-Box FINISHED %.2fs (max=%.1f km/h)", (double)s_st.elapsed_s, (double)s_st.max_speed_kmh);
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
    if (s_mutex == NULL) {
        memset(out, 0, sizeof(*out));
        return;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *out = s_st;
        xSemaphoreGive(s_mutex);
    }
}
