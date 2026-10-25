/*
 * pbox.h - P-Box / P-Gear 性能测试逻辑（README §4.4）
 * 状态机：READY --起步条件--> RUNNING --目标进阶/超时--> FINISHED
 * 目标区间：PBOX_TARGETS_KMH 顺序进阶（默认 60 → 100 km/h）
 * 成绩历史：每次 FINISHED 追加一条记录，NVS 环形保存最近 PBOX_HISTORY_MAX 条
 * 交互：双击 = 清当前成绩（历史保留）；三击 = 历届成绩清单（UI 层实现）
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "config.h"         /* PBOX_HISTORY_MAX / PBOX_TARGETS_KMH */

typedef enum {
    PBOX_READY = 0,     /* 等待起步条件（踩油门即自动进入 RUNNING） */
    PBOX_ARMED,         /* 预留：手动预位（当前未使用） */
    PBOX_RUNNING,       /* 计时中（目标进阶中） */
    PBOX_FINISHED,      /* 本次测试完成（成绩已入历史） */
} pbox_state_t;

typedef struct {
    pbox_state_t state;
    uint8_t target_idx;         /* 当前目标区间索引（0=60, 1=100） */
    float target_kmh;           /* 当前目标速度 */
    float elapsed_s;            /* 已用时间（RUNNING 起） */
    float max_speed_kmh;        /* 测试内最大速度 */
    bool can_start;             /* 启动条件当前是否满足（预留） */
    uint64_t t0_us;             /* RUNNING 起始时刻（esp_timer） */
    float t_0_60;               /* 0-60 km/h 耗时（<=0 表示未测出） */
    float t_0_100;              /* 0-100 km/h 耗时（<=0 表示未测出） */
    float t_400m;               /* 400m 耗时（<=0 表示未测出，尚未实现） */
    float slope_pct;            /* 测试平均坡度（百分比，尚未实现） */
    float peak_g;               /* 测试期间（仅 RUNNING）峰值 G 值 */
    float distance_m;           /* 累计加速里程（GNSS 速度梯形积分） */
} pbox_status_t;

/** @brief 单次测试成绩记录（历届成绩清单条目，新→旧排列） */
typedef struct {
    float t_0_60;               /* 0-60 km/h 耗时（<=0 未测出） */
    float t_0_100;              /* 0-100 km/h 耗时（<=0 未测出） */
    float distance_m;           /* 本次测试里程（m） */
    float max_speed_kmh;        /* 本次最高速度（km/h） */
    uint32_t utc_sec;           /* 测试完成时刻 UTC epoch（0=未对时） */
} pbox_record_t;

esp_err_t pbox_init(void);

/** @brief 双击：清空当前成绩（计时/0-60/0-100/里程/峰值），**历史记录保留** */
void pbox_clear_current(void);

/** @brief 周期调用（≥10Hz）：车速 + 线性加速度峰值(g) + 当前 GNSS UTC
 *  @param utc_sec 传 0 表示本次无有效时间（历史记录沿用最近一次有效值）
 */
void pbox_update(float speed_kmh, float acc_x_g, uint32_t utc_sec);

/** @brief 获取实时状态快照（线程安全） */
void pbox_get_status(pbox_status_t *out);

/**
 * @brief 获取历届成绩快照（线程安全）
 * @param out 输出数组（out[0] = 最新一条）
 * @param max 最多拷贝条数
 * @return 实际条数
 */
int pbox_history_snapshot(pbox_record_t *out, int max);
