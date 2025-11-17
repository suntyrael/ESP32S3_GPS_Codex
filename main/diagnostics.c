#include "diagnostics.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "diag";

void diagnostics_init(void) {
    ESP_LOGI(TAG, "Diagnostics subsystem ready");
}

static void print_satellites(const sensors_state_t *state) {
    for (int i = 0; i < state->gnss.sats_in_view && i < GNSS_MAX_SATELLITES; ++i) {
        const gnss_satellite_t *sat = &state->gnss.satellites[i];
        ESP_LOGD(TAG, "SAT %02u CONST=%d CN0=%.1f STATUS=%d", sat->id, sat->constellation, sat->cn0_dbhz, sat->status);
    }
}

void diagnostics_report_boot(const sensors_state_t *state, uint32_t uptime_ms) {
    ESP_LOGI(TAG, "[BOOT][T=%ums] GNSS sats=%u fix=%d lat=%.5f lon=%.5f", uptime_ms,
             state->gnss.sats_in_view, state->gnss.fix_valid, state->gnss.latitude_deg, state->gnss.longitude_deg);
    ESP_LOGI(TAG, "IMU ACC(%.2f,%.2f,%.2f) GRAV(%.2f,%.2f,%.2f)",
             state->imu.linear_accel_g.x, state->imu.linear_accel_g.y, state->imu.linear_accel_g.z,
             state->imu.gravity_g.x, state->imu.gravity_g.y, state->imu.gravity_g.z);
    ESP_LOGI(TAG, "BARO %.1fhPa ALT=%.1fm TEMP=%.1fC", state->baro.pressure_hpa, state->baro.altitude_m, state->baro.temperature.temperature_c);
    ESP_LOGI(TAG, "POWER %.2fV %u%% charging=%d", state->power.battery_voltage_v, state->power.battery_percent, state->power.charging);
    print_satellites(state);
}

void diagnostics_report_heartbeat(const sensors_state_t *state, uint32_t uptime_ms) {
    ESP_LOGI(TAG, "[HB][T=%ums] sats=%u speed=%.1fkm/h battery=%u%%", uptime_ms, state->gnss.sats_in_use, state->gnss.speed_kmh, state->power.battery_percent);
}

void diagnostics_trigger_event(const char *event_name, uint32_t duration_ms) {
    ESP_LOGI(TAG, "[INPUT] %s duration=%ums", event_name, duration_ms);
}

