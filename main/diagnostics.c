#include "diagnostics.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * 此文件根据 suntyrael/ESP32S3_GPS_Codex 中的 diagnostics.c 改写。
 * 主要改动为：
 * 1. diagnostics_report_heartbeat 输出完整的传感器状态，包括 GNSS、IMU、MAG、BARO 和电源信息；
 * 2. 保持 boot 报告函数与原仓库一致。
 */

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
        ESP_LOGI(TAG, "SAT %02u CONST=%d CN0=%.1f STATUS=%s", sat->id, sat->constellation, sat->cn0_dbhz,
                 sat_status_str(sat->status));
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
    // 输出完整的外设状态，与引导报告格式一致
    ESP_LOGI(TAG, "[HB][T=%ums]", uptime_ms);
    // GNSS状态：GPS 固定、可见卫星/使用卫星、经纬度、高度、速度
    ESP_LOGI(TAG, "GNSS: fix=%d view=%u use=%u lat=%.5f lon=%.5f alt=%.1fm spd=%.1fkm/h", state->gnss.fix_valid,
             state->gnss.sats_in_view, state->gnss.sats_in_use, state->gnss.latitude_deg, state->gnss.longitude_deg,
             state->gnss.altitude_m, state->gnss.speed_kmh);
    // 定位精度指标
    ESP_LOGI(TAG, "DOP: HDOP=%.2f VDOP=%.2f PDOP=%.2f", state->gnss.hdop, state->gnss.vdop, state->gnss.pdop);
    // IMU 传感器
    ESP_LOGI(TAG, "IMU: LIN(%.2f,%.2f,%.2f) GRAV(%.2f,%.2f,%.2f) TEMP=%.1fC",
             state->imu.linear_accel_g.x, state->imu.linear_accel_g.y, state->imu.linear_accel_g.z,
             state->imu.gravity_g.x, state->imu.gravity_g.y, state->imu.gravity_g.z, state->imu.temperature.temperature_c);
    // 磁力计
    ESP_LOGI(TAG, "MAG: (%.1f,%.1f,%.1f) TEMP=%.1fC", state->mag.magnetic_ut.x, state->mag.magnetic_ut.y,
             state->mag.magnetic_ut.z, state->mag.temperature.temperature_c);
    // 气压计
    ESP_LOGI(TAG, "BARO: %.1fhPa ALT=%.1fm TEMP=%.1fC", state->baro.pressure_hpa, state->baro.altitude_m,
             state->baro.temperature.temperature_c);
    // 电源状态
    ESP_LOGI(TAG, "POWER: %.2fV %u%% charging=%d", state->power.battery_voltage_v, state->power.battery_percent,
             state->power.charging);
    // 输出卫星信息
    print_satellites(state);
}

void diagnostics_trigger_event(const char *event_name, uint32_t duration_ms) {
    ESP_LOGI(TAG, "[INPUT] %s duration=%ums", event_name, duration_ms);
}