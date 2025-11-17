#include "sensors.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"

static const char *TAG = "sensors";
static sensors_state_t s_state;

void sensors_init(void) {
    memset(&s_state, 0, sizeof(s_state));
    ESP_LOGI(TAG, "Initializing sensors and loading calibration data");
    // 这里应初始化 I2C/SPI/UART 以及各传感器驱动。
}

static void update_dummy_gnss(void) {
    s_state.gnss.fix_valid = true;
    s_state.gnss.latitude_deg = 22.54321;
    s_state.gnss.longitude_deg = 113.9421;
    s_state.gnss.altitude_m = 125.0f;
    s_state.gnss.speed_kmh = 32.5f;
    s_state.gnss.hdop = 0.9f;
    s_state.gnss.vdop = 1.1f;
    s_state.gnss.pdop = 1.4f;
    s_state.gnss.sats_in_view = 12;
    s_state.gnss.sats_in_use = 8;
    for (int i = 0; i < s_state.gnss.sats_in_view && i < GNSS_MAX_SATELLITES; ++i) {
        s_state.gnss.satellites[i].id = i + 1;
        s_state.gnss.satellites[i].constellation = (i % 4);
        s_state.gnss.satellites[i].cn0_dbhz = 35.0f + i;
        s_state.gnss.satellites[i].status = (i % 3);
    }
}

static void update_dummy_imu(void) {
    s_state.imu.linear_accel_g.x = 0.01f;
    s_state.imu.linear_accel_g.y = 0.02f;
    s_state.imu.linear_accel_g.z = 0.98f;
    s_state.imu.gravity_g.z = 1.0f;
    s_state.imu.temperature.temperature_c = 36.0f;
    s_state.mag.magnetic_ut.x = 32.0f;
    s_state.mag.temperature.temperature_c = 35.0f;
    s_state.baro.altitude_m = 126.0f;
    s_state.baro.pressure_hpa = 1013.5f;
    s_state.baro.temperature.temperature_c = 34.8f;
}

static void update_dummy_power(void) {
    s_state.power.battery_voltage_v = 3.95f;
    s_state.power.battery_percent = 78;
    s_state.power.charging = false;
}

void sensors_update(void) {
    update_dummy_gnss();
    update_dummy_imu();
    update_dummy_power();
}

void sensors_get_state(sensors_state_t *state_out) {
    if (!state_out) {
        return;
    }
    memcpy(state_out, &s_state, sizeof(sensors_state_t));
}

