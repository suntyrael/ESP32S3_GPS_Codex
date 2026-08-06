#include "sensors.h"
#include "config.h"
#include "i2c_bus.h"
#include "lsm6dsr.h"
#include "lis2mdl.h"
#include "bmp388.h"
#include "battery.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "sensors";

static SemaphoreHandle_t s_mutex = NULL;
static sensors_state_t s_state = { 0 };
static lsm6dsr_handle_t s_imu = NULL;
static lis2mdl_handle_t s_mag = NULL;
static bmp388_handle_t s_baro = NULL;

esp_err_t sensors_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        ESP_LOGE(TAG, "mutex 创建失败");
        return ESP_ERR_NO_MEM;
    }

    i2c_master_bus_handle_t bus = NULL;
    esp_err_t ret = i2c_bus_init(&bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 总线初始化失败，传感器全部不可用");
        return ret;
    }

    /* 三个 I2C 传感器：失败各自降级（valid=false），不阻塞其他 */
    s_imu = NULL;
    if (lsm6dsr_init(bus, &s_imu) == ESP_OK && s_imu != NULL) {
        s_state.imu.valid = true;
    }
    s_mag = NULL;
    if (lis2mdl_init(bus, &s_mag) == ESP_OK && s_mag != NULL) {
        s_state.mag.valid = true;
    }
    s_baro = NULL;
    if (bmp388_init(bus, &s_baro) == ESP_OK && s_baro != NULL) {
        s_state.baro.valid = true;
    }

    if (battery_init() == ESP_OK) {
        s_state.battery.valid = true;
    }

    ESP_LOGI(TAG, "sensors_init 完成: IMU=%s MAG=%s BARO=%s BAT=%s",
             s_state.imu.valid ? "OK" : "FAIL",
             s_state.mag.valid ? "OK" : "FAIL",
             s_state.baro.valid ? "OK" : "FAIL",
             s_state.battery.valid ? "OK" : "FAIL");
    return ESP_OK;
}

esp_err_t sensors_update(void)
{
    sensors_state_t st = { 0 };
    st.imu.valid = s_state.imu.valid;
    st.mag.valid = s_state.mag.valid;
    st.baro.valid = s_state.baro.valid;
    st.battery.valid = s_state.battery.valid;

    if (s_imu != NULL) {
        lsm6dsr_data_t d;
        if (lsm6dsr_read(s_imu, &d) == ESP_OK) {
            for (int i = 0; i < 3; i++) {
                st.imu.accel_mg[i] = d.accel_mg[i];
                st.imu.gyro_mdps[i] = d.gyro_mdps[i];
            }
            st.imu.temp_c = d.temp_c;
            st.imu.valid = true;
        } else {
            st.imu.valid = false;
        }
    }
    if (s_mag != NULL) {
        lis2mdl_data_t d;
        if (lis2mdl_read(s_mag, &d) == ESP_OK) {
            for (int i = 0; i < 3; i++) {
                st.mag.mag_mgauss[i] = d.mag_mgauss[i];
            }
            st.mag.temp_c = d.temp_c;
            st.mag.valid = true;
        } else {
            st.mag.valid = false;
        }
    }
    if (s_baro != NULL) {
        bmp388_data_t d;
        if (bmp388_read(s_baro, &d) == ESP_OK) {
            st.baro.temp_c = d.temp_c;
            st.baro.pressure_hpa = d.pressure_hpa;
            st.baro.altitude_m = d.altitude_m;
            st.baro.valid = true;
        } else {
            st.baro.valid = false;
        }
    }
    if (s_state.battery.valid) {
        battery_data_t d;
        if (battery_read(&d) == ESP_OK) {
            st.battery.voltage_v = d.voltage_v;
            st.battery.percent = d.percent;
            st.battery.saturated = d.saturated;
            st.battery.charging = d.charging;
            st.battery.valid = true;
        } else {
            st.battery.valid = false;
        }
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_state = st;
        xSemaphoreGive(s_mutex);
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

void sensors_get_state(sensors_state_t *out)
{
    if (out == NULL) {
        return;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *out = s_state;
        xSemaphoreGive(s_mutex);
    }
}

bool sensors_all_ready(void)
{
    sensors_state_t st;
    sensors_get_state(&st);
    return st.imu.valid && st.mag.valid && st.baro.valid && st.battery.valid;
}
