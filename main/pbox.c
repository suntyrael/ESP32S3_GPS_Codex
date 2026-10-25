/*
 * pbox.c - P-Box / P-Gear 性能测试实现
 *
 * 状态机：READY --起步条件--> RUNNING --末端目标达成/超时--> FINISHED
 *         双击（UI 层）--> READY + 清当前成绩（历史记录保留）
 *
 * 统计口径：
 *   · 0-60 / 0-100：首次跨过 PBOX_SPLIT_* 门限的 elapsed_s 锁存；
 *   · 里程：GNSS Doppler 速度梯形积分（dt 上限 PBOX_DIST_DT_MAX_S 丢弃异常间隔）；
 *   · PEAK G：仅在 RUNNING 内统计，FINISHED 后冻结（停车踩油门不会污染成绩）；
 *   · 目标进阶：60 达成后继续计时跑 100，未达末端目标则 PBOX_RUN_TIMEOUT_S 强制收尾。
 *
 * 成绩历史：FINISHED 时追加一条（0-60 / 0-100 / 里程 / 最高速 / 完成 UTC），
 *          NVS 环形保存最近 PBOX_HISTORY_MAX 条（namespace "pbox", key "hist"）。
 *          写 Flash 在持锁区外执行，避免阻塞 100 Hz 采样。
 */
#include "pbox.h"
#include "config.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "pbox";

#define PBOX_NVS_NS         "pbox"
#define PBOX_NVS_KEY_HIST   "hist"

/* NVS blob 布局（带版本号，字段变更必须递增 PBOX_HISTORY_VER） */
typedef struct {
    uint32_t ver;
    uint32_t cnt;
    pbox_record_t rec[PBOX_HISTORY_MAX];
} pbox_hist_blob_t;

static SemaphoreHandle_t s_mutex = NULL;
static pbox_status_t s_st = { 0 };

static pbox_record_t s_hist[PBOX_HISTORY_MAX];  /* 新→旧 */
static int s_hist_cnt = 0;
static bool s_hist_dirty = false;

static uint64_t s_prev_us = 0;                  /* 上一帧时间基（里程积分用） */
static float s_prev_speed_kmh = 0.0f;
static uint32_t s_utc_sec = 0;                  /* 最近一帧有效 GNSS UTC */

static const float s_targets[] = PBOX_TARGETS_KMH;
#define PBOX_TARGET_CNT  (sizeof(s_targets) / sizeof(s_targets[0]))

/* ==================== 成绩历史：NVS 持久化 ==================== */
static void hist_load(void)
{
    pbox_hist_blob_t blob;
    memset(&blob, 0, sizeof(blob));

    nvs_handle_t handle;
    if (nvs_open(PBOX_NVS_NS, NVS_READONLY, &handle) != ESP_OK) {
        ESP_LOGI(TAG, "history: empty (namespace not created yet)");
        return;
    }
    size_t len = sizeof(blob);
    esp_err_t err = nvs_get_blob(handle, PBOX_NVS_KEY_HIST, &blob, &len);
    nvs_close(handle);

    if (err != ESP_OK || len != sizeof(blob) || blob.ver != PBOX_HISTORY_VER) {
        ESP_LOGW(TAG, "history: blob invalid (err=%s len=%u ver=%u), start empty",
                 esp_err_to_name(err), (unsigned)len, (unsigned)blob.ver);
        return;
    }

    int cnt = (int)blob.cnt;
    if (cnt < 0) {
        cnt = 0;
    } else if (cnt > PBOX_HISTORY_MAX) {
        cnt = PBOX_HISTORY_MAX;
    }
    memcpy(s_hist, blob.rec, sizeof(s_hist));
    s_hist_cnt = cnt;
    ESP_LOGI(TAG, "history loaded: %d record(s)", s_hist_cnt);
}

/* 锁内只拷快照并清 dirty，Flash 写在锁外完成（避免阻塞高频 pbox_update） */
static void hist_persist_if_dirty(void)
{
    pbox_hist_blob_t blob;
    bool dirty = false;

    if (s_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (s_hist_dirty) {
            s_hist_dirty = false;
            dirty = true;
            blob.ver = PBOX_HISTORY_VER;
            blob.cnt = (uint32_t)s_hist_cnt;
            memcpy(blob.rec, s_hist, sizeof(blob.rec));
        }
        xSemaphoreGive(s_mutex);
    }
    if (!dirty) {
        return;
    }

    nvs_handle_t handle;
    if (nvs_open(PBOX_NVS_NS, NVS_READWRITE, &handle) != ESP_OK) {
        ESP_LOGW(TAG, "history: nvs_open RW failed, record only in RAM");
        return;
    }
    if (nvs_set_blob(handle, PBOX_NVS_KEY_HIST, &blob, sizeof(blob)) == ESP_OK) {
        nvs_commit(handle);
        ESP_LOGI(TAG, "history saved (%u record(s))", (unsigned)blob.cnt);
    } else {
        ESP_LOGW(TAG, "history: nvs_set_blob failed");
    }
    nvs_close(handle);
}

/* 调用方必须持有 s_mutex */
static void hist_push_locked(void)
{
    /* 有效门限：至少测出 0-60/0-100 之一，或跑出有效里程，否则视为误触发丢弃 */
    if (s_st.t_0_60 <= 0.0f && s_st.t_0_100 <= 0.0f &&
        s_st.distance_m < PBOX_HISTORY_MIN_DIST_M) {
        ESP_LOGW(TAG, "run discarded: no valid split (dist=%.1fm)", (double)s_st.distance_m);
        return;
    }

    int last = (s_hist_cnt < PBOX_HISTORY_MAX) ? s_hist_cnt : (PBOX_HISTORY_MAX - 1);
    for (int i = last; i > 0; i--) {
        s_hist[i] = s_hist[i - 1];
    }
    s_hist[0].t_0_60 = s_st.t_0_60;
    s_hist[0].t_0_100 = s_st.t_0_100;
    s_hist[0].distance_m = s_st.distance_m;
    s_hist[0].max_speed_kmh = s_st.max_speed_kmh;
    s_hist[0].utc_sec = s_utc_sec;
    if (s_hist_cnt < PBOX_HISTORY_MAX) {
        s_hist_cnt++;
    }
    s_hist_dirty = true;

    ESP_LOGI(TAG, "history push (%d/%d): 0-60=%.2fs 0-100=%.2fs dist=%.1fm max=%.1fkm/h",
             s_hist_cnt, PBOX_HISTORY_MAX,
             (double)s_hist[0].t_0_60, (double)s_hist[0].t_0_100,
             (double)s_hist[0].distance_m, (double)s_hist[0].max_speed_kmh);
}

/* ==================== 生命周期 ==================== */
esp_err_t pbox_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }
    memset(&s_st, 0, sizeof(s_st));
    s_st.state = PBOX_READY;
    s_st.target_idx = 0;
    s_st.target_kmh = s_targets[0];

    hist_load();

    ESP_LOGI(TAG, "pbox init ok (targets: %d, history: %d/%d)",
             (int)PBOX_TARGET_CNT, s_hist_cnt, PBOX_HISTORY_MAX);
    return ESP_OK;
}

void pbox_clear_current(void)
{
    if (s_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "clear current: mutex timeout");
        return;
    }
    s_st.state = PBOX_READY;
    s_st.target_idx = 0;
    s_st.target_kmh = s_targets[0];
    s_st.elapsed_s = 0.0f;
    s_st.max_speed_kmh = 0.0f;
    s_st.peak_g = 0.0f;
    s_st.t_0_60 = 0.0f;
    s_st.t_0_100 = 0.0f;
    s_st.t_400m = 0.0f;
    s_st.slope_pct = 0.0f;
    s_st.distance_m = 0.0f;
    s_st.t0_us = 0;
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "current result cleared (history kept: %d record(s))", s_hist_cnt);
}

/* ==================== 周期更新 ==================== */
void pbox_update(float speed_kmh, float acc_x_g, uint32_t utc_sec)
{
    if (s_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }

    uint64_t now = esp_timer_get_time();
    float dt = (s_prev_us != 0 && now > s_prev_us)
               ? (float)(now - s_prev_us) / 1000000.0f : 0.0f;
    if (dt > PBOX_DIST_DT_MAX_S) {
        dt = 0.0f;      /* 任务停顿/唤醒后的异常间隔，直接丢弃 */
    }
    if (utc_sec != 0 && utc_sec != s_utc_sec) {
        s_utc_sec = utc_sec;
    }

    switch (s_st.state) {
    case PBOX_READY:
    case PBOX_ARMED: {
        /* 起步条件：车速离开零点，或深踩油门产生向前推力 */
        bool launch_by_spd = (speed_kmh >= 1.5f && speed_kmh < 15.0f);
        bool launch_by_acc = (acc_x_g > 0.25f && speed_kmh >= 0.8f);

        if (launch_by_spd || launch_by_acc) {
            s_st.state = PBOX_RUNNING;
            s_st.t0_us = now;
            s_st.target_idx = 0;
            s_st.target_kmh = s_targets[0];
            s_st.elapsed_s = 0.0f;
            s_st.max_speed_kmh = speed_kmh;
            s_st.t_0_60 = 0.0f;
            s_st.t_0_100 = 0.0f;
            s_st.t_400m = 0.0f;
            s_st.slope_pct = 0.0f;
            s_st.distance_m = 0.0f;
            s_st.peak_g = 0.0f;
            ESP_LOGI(TAG, "LAUNCH! spd=%.1f acc=%.2fg", (double)speed_kmh, (double)acc_x_g);
        }
        break;
    }
    case PBOX_RUNNING: {
        s_st.elapsed_s = (float)(now - s_st.t0_us) / 1000000.0f;

        /* PEAK G：仅在测试计时期间统计（FINISHED 后冻结） */
        float abs_g = fabsf(acc_x_g);
        if (abs_g > s_st.peak_g) {
            s_st.peak_g = abs_g;
        }

        /* 里程：速度梯形积分（km/h → m） */
        if (dt > 0.0f) {
            s_st.distance_m += (s_prev_speed_kmh + speed_kmh) * 0.5f / 3.6f * dt;
        }

        /* 防误触发保护：起步后 2.5 秒内车速依然 < 2.0 km/h（未真正移动），回 READY */
        if (s_st.elapsed_s > 2.5f && speed_kmh < 2.0f) {
            s_st.state = PBOX_READY;
            s_st.elapsed_s = 0.0f;
            s_st.max_speed_kmh = 0.0f;
            s_st.distance_m = 0.0f;
            s_st.peak_g = 0.0f;
            ESP_LOGW(TAG, "false launch timed out -> READY");
            break;
        }

        if (speed_kmh > s_st.max_speed_kmh) {
            s_st.max_speed_kmh = speed_kmh;
        }

        /* 0-60 km/h 锁存 */
        if (speed_kmh >= PBOX_SPLIT_60_KMH && s_st.t_0_60 <= 0.0f) {
            s_st.t_0_60 = s_st.elapsed_s;
            ESP_LOGI(TAG, "0-60 km/h: %.2fs", (double)s_st.t_0_60);
        }

        /* 0-100 km/h 锁存 */
        if (speed_kmh >= PBOX_SPLIT_100_KMH && s_st.t_0_100 <= 0.0f) {
            s_st.t_0_100 = s_st.elapsed_s;
            ESP_LOGI(TAG, "0-100 km/h: %.2fs", (double)s_st.t_0_100);
        }

        /* 目标进阶：60 达成后继续计时到 100；末端目标达成或超时收尾 */
        if (speed_kmh >= s_st.target_kmh) {
            if ((size_t)s_st.target_idx + 1u < PBOX_TARGET_CNT) {
                s_st.target_idx++;
                s_st.target_kmh = s_targets[s_st.target_idx];
                ESP_LOGI(TAG, "target advanced -> %.0f km/h", (double)s_st.target_kmh);
            } else {
                s_st.state = PBOX_FINISHED;
                hist_push_locked();
                ESP_LOGI(TAG, "FINISHED %.2fs (max=%.1f km/h, dist=%.1fm)",
                         (double)s_st.elapsed_s, (double)s_st.max_speed_kmh,
                         (double)s_st.distance_m);
            }
        } else if (s_st.elapsed_s > PBOX_RUN_TIMEOUT_S) {
            s_st.state = PBOX_FINISHED;
            hist_push_locked();
            ESP_LOGW(TAG, "FINISHED by timeout %.1fs (max=%.1f km/h, dist=%.1fm)",
                     (double)s_st.elapsed_s, (double)s_st.max_speed_kmh,
                     (double)s_st.distance_m);
        }
        break;
    }
    default:
        break;
    }

    s_prev_us = now;
    s_prev_speed_kmh = speed_kmh;
    xSemaphoreGive(s_mutex);

    hist_persist_if_dirty();    /* 锁外写 Flash */
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

int pbox_history_snapshot(pbox_record_t *out, int max)
{
    if (out == NULL || max <= 0 || s_mutex == NULL) {
        return 0;
    }
    int n = 0;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        n = (s_hist_cnt < max) ? s_hist_cnt : max;
        memcpy(out, s_hist, (size_t)n * sizeof(pbox_record_t));
        xSemaphoreGive(s_mutex);
    }
    return n;
}
