#include "diagnostics.h"
#include "config.h"
#include "sensors.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DIAG";

static int64_t s_t0_ms = 0;

static void print_state(bool full)
{
    sensors_state_t st;
    sensors_get_state(&st);

    char line[512];
    int len = 0;
    len += snprintf(line + len, sizeof(line) - (size_t)len,
                    "[DIAG][T=%lldms]\n", (long long)(esp_timer_get_time() / 1000 - s_t0_ms));

    /* GNSS：阶段 3 接入 */
    len += snprintf(line + len, sizeof(line) - (size_t)len, "GNSS: N/A\n");

    if (st.imu.valid) {
        len += snprintf(line + len, sizeof(line) - (size_t)len,
                        "IMU: ACC(%.2f,%.2f,%.2f) GYRO(%.1f,%.1f,%.1f)\n",
                        st.imu.accel_mg[0] / 1000.0f, st.imu.accel_mg[1] / 1000.0f, st.imu.accel_mg[2] / 1000.0f,
                        st.imu.gyro_mdps[0], st.imu.gyro_mdps[1], st.imu.gyro_mdps[2]);
    } else {
        len += snprintf(line + len, sizeof(line) - (size_t)len, "IMU: FAIL\n");
    }
    if (st.mag.valid) {
        len += snprintf(line + len, sizeof(line) - (size_t)len,
                        "MAG: (%.1f,%.1f,%.1f)\n",
                        st.mag.mag_mgauss[0], st.mag.mag_mgauss[1], st.mag.mag_mgauss[2]);
    } else {
        len += snprintf(line + len, sizeof(line) - (size_t)len, "MAG: FAIL\n");
    }
    if (st.baro.valid) {
        len += snprintf(line + len, sizeof(line) - (size_t)len,
                        "BARO: %.1fhPa\n", st.baro.pressure_hpa);
    } else {
        len += snprintf(line + len, sizeof(line) - (size_t)len, "BARO: FAIL\n");
    }

    len += snprintf(line + len, sizeof(line) - (size_t)len,
                    "TEMP: IMU=%.1f°C, BARO=%.1f°C, MAG=%.1f°C\n",
                    st.imu.valid ? st.imu.temp_c : 0.0f,
                    st.baro.valid ? st.baro.temp_c : 0.0f,
                    st.mag.valid ? st.mag.temp_c : 0.0f);
    len += snprintf(line + len, sizeof(line) - (size_t)len,
                    "BAT: %.2fV %d%% %s%s\n",
                    st.battery.valid ? st.battery.voltage_v : 0.0f,
                    st.battery.valid ? st.battery.percent : 0,
                    st.battery.valid ? (st.battery.saturated ? "SATURATED" : "") : "INVALID",
                    st.battery.valid && st.battery.charging ? " CHARGING" : "");
    len += snprintf(line + len, sizeof(line) - (size_t)len, "RESULT: %s",
                    sensors_all_ready() ? "OK" : "PARTIAL/FAIL");
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

    /* 启动自检：DIAG_BOOT_COUNT 次，每秒一次 */
    for (int i = 0; i < DIAG_BOOT_COUNT; i++) {
        print_state(true);
        vTaskDelay(pdMS_TO_TICKS(DIAG_BOOT_PERIOD_MS));
    }
    /* 心跳：每 5 秒 */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(DIAG_HEARTBEAT_MS));
        print_state(false);
    }
}
