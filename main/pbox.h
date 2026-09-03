/*
 * pbox.h - P-Box 性能测试逻辑（README §4.4）
 * 启动条件：GPS 速度 < 1 km/h 且 IMU X 轴线性加速度 > 阈值（默认 0.15g）
 * 目标区间：0-60 / 0-100 km/h（config.h 可调）
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    PBOX_READY = 0,     /* 等待短按进入 ARMED */
    PBOX_ARMED,         /* 等待启动条件满足（自动 RUNNING） */
    PBOX_RUNNING,       /* 计时中 */
    PBOX_FINISHED,      /* 完成 */
} pbox_state_t;

typedef struct {
    pbox_state_t state;
    uint8_t target_idx;         /* 目标区间索引 */
    float target_kmh;           /* 当前目标速度 */
    float elapsed_s;            /* 已用时间（RUNNING 起） */
    float max_speed_kmh;        /* 测试内最大速度 */
    bool can_start;             /* 启动条件当前是否满足（ARMED 时 UI 提示） */
    uint64_t t0_us;             /* RUNNING 起始时刻（esp_timer） */
    float t_0_60;               /* 0-60 km/h 耗时（<=0 表示未测出） */
    float t_0_100;              /* 0-100 km/h 耗时（<=0 表示未测出） */
    float t_400m;               /* 400m 耗时（<=0 表示未测出） */
    float slope_pct;            /* 测试平均坡度（百分比） */
    float peak_g;               /* 测试期间峰值 G 值 */
    float distance_m;           /* 累计加速距离 */
} pbox_status_t;

esp_err_t pbox_init(void);

/** @brief 短按：READY→ARMED / ARMED→READY / FINISHED→READY */
void pbox_arm(void);

/** @brief 周期调用（≥10Hz）：速度 + IMU X 轴线性加速度(g) */
void pbox_update(float speed_kmh, float acc_x_g);

void pbox_get_status(pbox_status_t *out);
