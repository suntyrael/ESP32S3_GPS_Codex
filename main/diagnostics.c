#include "diagnostics.h"
#include "config.h"
#include "sensors.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DIAG";

static int64_t s_t0_ms = 0;

/* 本地时间（UTC+8）：GPS 同步后显示真实日期；未同步返回空串（不重复显示运行时长） */
static void format_local_time(char *buf, size_t sz)
{
    time_t now = time(NULL);
    if (now < 1600000000) {         /* 2020-09 之前视为未同步（time()=开机秒数） */
        buf[0] = '\0';
        return;
    }
    now += 8 * 3600;                /* UTC+8 手动偏移，避免依赖 TZ 环境 */
    struct tm *t = gmtime(&now);
    if (t == NULL) {
        buf[0] = '\0';
        return;
    }
    strftime(buf, sz, "%Y-%m-%d %H:%M:%S", t);
}

static void print_state(bool full)
{
    sensors_state_t st;
    sensors_get_state(&st);

    char line[512];
    int len = 0;
    char tstr[32];
    format_local_time(tstr, sizeof(tstr));
    long long ms = (long long)(esp_timer_get_time() / 1000 - s_t0_ms);
    if (tstr[0] != '\0') {
        len += snprintf(line + len, sizeof(line) - (size_t)len,
                        "[DIAG][T=%lldms][%s]\n", ms, tstr);
    } else {
        len += snprintf(line + len, sizeof(line) - (size_t)len, "[DIAG][T=%lldms]\n", ms);
    }

    /* GNSS：阶段 3 接入 */
    len += snprintf(line + len, sizeof(line) - (size_t)len, "GNSS: N/A\n");

    if (st.imu.valid) {
        len += snprintf(line + len, sizeof(line) - (size_t)len,
                        "IMU: ACC(%.2fg,%.2fg,%.2fg) LIN(%.2fg,%.2fg,%.2fg) GYRO(%.1f,%.1f,%.1fdps)%s\n",
                        st.imu.accel_mg[0] / 1000.0f, st.imu.accel_mg[1] / 1000.0f, st.imu.accel_mg[2] / 1000.0f,
                        st.imu.lin_mg[0] / 1000.0f, st.imu.lin_mg[1] / 1000.0f, st.imu.lin_mg[2] / 1000.0f,
                        st.imu.gyro_mdps[0] / 1000.0f, st.imu.gyro_mdps[1] / 1000.0f, st.imu.gyro_mdps[2] / 1000.0f,
                        st.imu.fails > 0 ? " (stale)" : "");
    } else {
        len += snprintf(line + len, sizeof(line) - (size_t)len, "IMU: FAIL\n");
    }
    if (st.mag.valid) {
        len += snprintf(line + len, sizeof(line) - (size_t)len,
                        "MAG: (%.1f,%.1f,%.1f mG)%s\n",
                        st.mag.mag_mgauss[0], st.mag.mag_mgauss[1], st.mag.mag_mgauss[2],
                        st.mag.fails > 0 ? " (stale)" : "");
    } else {
        len += snprintf(line + len, sizeof(line) - (size_t)len, "MAG: FAIL\n");
    }
    if (st.baro.valid) {
        len += snprintf(line + len, sizeof(line) - (size_t)len,
                        "BARO: %.1f hPa%s\n", st.baro.pressure_hpa,
                        st.baro.fails > 0 ? " (stale)" : "");
    } else {
        len += snprintf(line + len, sizeof(line) - (size_t)len, "BARO: FAIL\n");
    }

    len += snprintf(line + len, sizeof(line) - (size_t)len,
                    "TEMP: IMU=%.1f°C, BARO=%.1f°C, MAG=%.1f°C\n",
                    st.imu.valid ? st.imu.temp_c : 0.0f,
                    st.baro.valid ? st.baro.temp_c : 0.0f,
                    st.mag.valid ? st.mag.temp_c : 0.0f);
    len += snprintf(line + len, sizeof(line) - (size_t)len,
                    "BAT: %.2fV (ADC=%umV) %d%% %s%s\n",
                    st.battery.valid ? st.battery.voltage_v : 0.0f,
                    st.battery.valid ? st.battery.adc_mv : 0,
                    st.battery.valid ? st.battery.percent : 0,
                    st.battery.valid ? (st.battery.saturated ? "SATURATED" : "") : "INVALID",
                    st.battery.valid && st.battery.charging ? " CHARGING" : "");
    /* RESULT 与显示同源（同一快照 st），避免双快照竞态误报 */
    bool all_ok = st.imu.valid && st.mag.valid && st.baro.valid && st.battery.valid;
    len += snprintf(line + len, sizeof(line) - (size_t)len, "RESULT: %s",
                    all_ok ? "OK" : "PARTIAL/FAIL");
    if (full) {
        ESP_LOGI(TAG, "\n%s", line);
    } else {
        ESP_LOGI(TAG, "%s", line);
    }
}

void diagnostics_report_boot(void)
{
    print_state(true);
}

void diagnostics_task(void *arg)
{
    (void)arg;
    s_t0_ms = esp_timer_get_time() / 1000;

    /* 启动自检：DIAG_BOOT_COUNT 次，每秒一次（先采样一次再打印，避免首行全 0） */
    for (int i = 0; i < DIAG_BOOT_COUNT; i++) {
        vTaskDelay(pdMS_TO_TICKS(DIAG_BOOT_PERIOD_MS));
        print_state(true);
    }
    /* 心跳：每 5 秒 */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(DIAG_HEARTBEAT_MS));
        print_state(false);
    }
}
