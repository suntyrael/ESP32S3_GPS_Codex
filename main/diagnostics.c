#include "diagnostics.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "diag";

void diagnostics_init(void) {
    ESP_LOGI(TAG, "Diagnostics subsystem ready");
}

static const char *sat_status_str(gnss_sat_status_t status) {
    switch (status) {
        case GNSS_SAT_STATUS_USED:
            return "USE";
        case GNSS_SAT_STATUS_TRACKING:
            return "TRK";
        default:
            return "SRCH";
    }
}

static void print_satellites(const sensors_state_t *state) {
    for (int i = 0; i < GNSS_MAX_SATELLITES; ++i) {
        const gnss_satellite_t *sat = &state->gnss.satellites[i];
        if (sat->id == 0) {
            continue;
        }
        ESP_LOGI(TAG, "SAT %02u CONST=%d CN0=%.1f STATUS=%s", sat->id, sat->constellation, sat->cn0_dbhz, sat_status_str(sat->status));
    }
}

void diagnostics_report_boot(const sensors_state_t *state, uint32_t uptime_ms) {
    ESP_LOGI(TAG, "[DIAG][T=%ums]", uptime_ms);
    ESP_LOGI(TAG, "GNSS: fix=%d view=%u use=%u lat=%.5f lon=%.5f alt=%.1fm spd=%.1fkm/h", state->gnss.fix_valid,
             state->gnss.sats_in_view, state->gnss.sats_in_use, state->gnss.latitude_deg, state->gnss.longitude_deg,
             state->gnss.altitude_m, state->gnss.speed_kmh);
    ESP_LOGI(TAG, "DOP: HDOP=%.2f VDOP=%.2f PDOP=%.2f", state->gnss.hdop, state->gnss.vdop, state->gnss.pdop);
    ESP_LOGI(TAG, "IMU: LIN(%.2f,%.2f,%.2f) GRAV(%.2f,%.2f,%.2f) TEMP=%.1fC",
             state->imu.linear_accel_g.x, state->imu.linear_accel_g.y, state->imu.linear_accel_g.z,
             state->imu.gravity_g.x, state->imu.gravity_g.y, state->imu.gravity_g.z, state->imu.temperature.temperature_c);
    ESP_LOGI(TAG, "MAG: (%.1f,%.1f,%.1f) TEMP=%.1fC", state->mag.magnetic_ut.x, state->mag.magnetic_ut.y,
             state->mag.magnetic_ut.z, state->mag.temperature.temperature_c);
    ESP_LOGI(TAG, "BARO: %.1fhPa ALT=%.1fm TEMP=%.1fC", state->baro.pressure_hpa, state->baro.altitude_m,
             state->baro.temperature.temperature_c);
    ESP_LOGI(TAG, "POWER: %.2fV %u%% charging=%d", state->power.battery_voltage_v, state->power.battery_percent,
             state->power.charging);
    print_satellites(state);
}

void diagnostics_report_heartbeat(const sensors_state_t *state, uint32_t uptime_ms) {
    ESP_LOGI(TAG, "[HB][T=%ums] sats=%u/%u speed=%.1fkm/h battery=%u%% temp=%.1fC",
             uptime_ms, state->gnss.sats_in_use, state->gnss.sats_in_view, state->gnss.speed_kmh,
             state->power.battery_percent, state->imu.temperature.temperature_c);
}

void diagnostics_trigger_event(const char *event_name, uint32_t duration_ms) {
    ESP_LOGI(TAG, "[INPUT] %s duration=%ums", event_name, duration_ms);
}

