#include "sensors.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "config.h"
#include "gnss.h"

static const char *TAG = "sensors";
static sensors_state_t s_state;
static SemaphoreHandle_t s_lock;
static StaticSemaphore_t s_lock_buffer;

void sensors_init(void) {
    memset(&s_state, 0, sizeof(s_state));
    s_lock = xSemaphoreCreateMutexStatic(&s_lock_buffer);
    ESP_LOGI(TAG, "Initializing sensors and loading calibration data");
    // TODO: 初始化 I2C/SPI 驱动并加载校准参数
    gnss_init();
}

void sensors_update(void) {
    if (!s_lock) {
        return;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    gnss_poll(&s_state.gnss);

    int64_t t_us = esp_timer_get_time();
    float t = (float)(t_us / 1000000.0);
    s_state.imu.linear_accel_g.x = 0.02f * sinf(t);
    s_state.imu.linear_accel_g.y = 0.02f * cosf(t * 0.5f);
    s_state.imu.linear_accel_g.z = 0.98f;
    s_state.imu.gravity_g.x = 0.0f;
    s_state.imu.gravity_g.y = 0.0f;
    s_state.imu.gravity_g.z = 1.0f;
    s_state.imu.gyro_dps.x = 0.5f * sinf(t * 0.2f);
    s_state.imu.gyro_dps.y = 0.4f * cosf(t * 0.25f);
    s_state.imu.gyro_dps.z = 0.1f;
    s_state.imu.temperature.temperature_c = 35.0f + 0.2f * sinf(t * 0.1f);

    s_state.mag.magnetic_ut.x = 32.0f;
    s_state.mag.magnetic_ut.y = 5.0f;
    s_state.mag.magnetic_ut.z = -42.0f;
    s_state.mag.temperature.temperature_c = 34.0f;

    s_state.baro.pressure_hpa = 1012.0f + 2.0f * sinf(t * 0.05f);
    s_state.baro.altitude_m = 120.0f + 0.5f * cosf(t * 0.07f);
    s_state.baro.temperature.temperature_c = 33.5f;

    s_state.power.battery_voltage_v = 3.8f + 0.05f * sinf(t * 0.01f);
    float pct = (s_state.power.battery_voltage_v - 3.5f) * 200.0f;
    if (pct < 0.0f) {
        pct = 0.0f;
    }
    if (pct > 100.0f) {
        pct = 100.0f;
    }
    s_state.power.battery_percent = (uint8_t)pct;
    s_state.power.charging = false;

    xSemaphoreGive(s_lock);
}

void sensors_get_state(sensors_state_t *state_out) {
    if (!state_out) {
        return;
    }
    if (!s_lock) {
        memset(state_out, 0, sizeof(*state_out));
        return;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(state_out, &s_state, sizeof(sensors_state_t));
        xSemaphoreGive(s_lock);
    } else {
        memset(state_out, 0, sizeof(*state_out));
    }
}

