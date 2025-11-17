#include "sensors.h"

#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_bit_defs.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"

#include "config.h"
#include "gnss.h"

static const char *TAG = "sensors";
static sensors_state_t s_state;
static SemaphoreHandle_t s_lock;
static StaticSemaphore_t s_lock_buffer;
static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_adc_cali;

typedef struct {
    vector3f_t accel_bias;
    vector3f_t gyro_bias;
    vector3f_t mag_bias;
    float mag_scale;
} sensor_calibration_t;

static sensor_calibration_t s_calibration;

#define LSM6DSR_ADDR        0x6A
#define LIS2MDL_ADDR        0x1E
#define BMP388_ADDR         0x77

typedef struct {
    uint16_t T1;
    uint16_t T2;
    int8_t T3;
    int16_t P1;
    int16_t P2;
    int8_t P3;
    int8_t P4;
    int16_t P5;
    int16_t P6;
    int8_t P7;
    int8_t P8;
    int16_t P9;
    int8_t P10;
    int8_t P11;
} bmp388_calib_t;

static bmp388_calib_t s_bmp_calib;
static float s_bmp_t_lin;

static esp_err_t i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(CONFIG_SENSOR_I2C_PORT, addr, &reg, 1, data, len, pdMS_TO_TICKS(50));
}

static esp_err_t i2c_write(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_master_write_to_device(CONFIG_SENSOR_I2C_PORT, addr, buf, sizeof(buf), pdMS_TO_TICKS(50));
}

static void load_calibration(void) {
    memset(&s_calibration, 0, sizeof(s_calibration));
    s_calibration.mag_scale = 1.0f;
    nvs_handle_t handle;
    if (nvs_open("cal", NVS_READONLY, &handle) != ESP_OK) {
        ESP_LOGW(TAG, "Calibration not found");
        return;
    }
    size_t size = sizeof(s_calibration);
    esp_err_t err = nvs_get_blob(handle, "imu", &s_calibration, &size);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load calibration (%d)", err);
        memset(&s_calibration, 0, sizeof(s_calibration));
        s_calibration.mag_scale = 1.0f;
    } else {
        ESP_LOGI(TAG, "Calibration loaded");
    }
    nvs_close(handle);
}

static void init_i2c(void) {
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_SENSOR_I2C_SDA,
        .scl_io_num = CONFIG_SENSOR_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = CONFIG_SENSOR_I2C_FREQ_HZ,
    };
    i2c_param_config(CONFIG_SENSOR_I2C_PORT, &cfg);
    i2c_driver_install(CONFIG_SENSOR_I2C_PORT, cfg.mode, 0, 0, 0);
}

static void init_adc(void) {
    adc_oneshot_unit_init_cfg_t cfg = {
        .unit_id = ADC_UNIT_2,
    };
    adc_oneshot_new_unit(&cfg, &s_adc_handle);
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_11,
    };
    adc_oneshot_config_channel(s_adc_handle, CONFIG_BATTERY_ADC_CHANNEL, &chan_cfg);
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cal_cfg = {
        .unit_id = ADC_UNIT_2,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_line_fitting(&cal_cfg, &s_adc_cali) != ESP_OK) {
        s_adc_cali = NULL;
    }
#endif
}

static esp_err_t lsm6dsr_init(void) {
    // Reset and configure default ODRs
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_write(LSM6DSR_ADDR, 0x12, 0x01));
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_write(LSM6DSR_ADDR, 0x10, 0x60)); // Accel 416Hz, 2g
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_write(LSM6DSR_ADDR, 0x11, 0x60)); // Gyro 416Hz, 2000dps
    return ESP_OK;
}

static void apply_orientation(vector3f_t *vec) {
    vec->x = -vec->x;
    vec->z = -vec->z;
}

static esp_err_t lsm6dsr_read(vector3f_t *accel_g, vector3f_t *gyro_dps, float *temp_c) {
    uint8_t buf[12];
    esp_err_t err = i2c_read(LSM6DSR_ADDR, 0x22, buf, sizeof(buf));
    if (err != ESP_OK) {
        return err;
    }
    int16_t gyro_raw[3];
    int16_t accel_raw[3];
    for (int i = 0; i < 3; ++i) {
        gyro_raw[i] = (int16_t)((buf[i * 2 + 1] << 8) | buf[i * 2]);
        accel_raw[i] = (int16_t)((buf[i * 2 + 7] << 8) | buf[i * 2 + 6]);
    }
    const float gyro_scale = 0.07f; // mdps->dps
    const float accel_scale = 0.000061f * 1000.0f; // mg->g
    gyro_dps->x = gyro_raw[0] * gyro_scale - s_calibration.gyro_bias.x;
    gyro_dps->y = gyro_raw[1] * gyro_scale - s_calibration.gyro_bias.y;
    gyro_dps->z = gyro_raw[2] * gyro_scale - s_calibration.gyro_bias.z;
    accel_g->x = accel_raw[0] * accel_scale - s_calibration.accel_bias.x;
    accel_g->y = accel_raw[1] * accel_scale - s_calibration.accel_bias.y;
    accel_g->z = accel_raw[2] * accel_scale - s_calibration.accel_bias.z;
    apply_orientation(accel_g);
    apply_orientation(gyro_dps);

    uint8_t temp_buf[2];
    if (i2c_read(LSM6DSR_ADDR, 0x20, temp_buf, sizeof(temp_buf)) == ESP_OK) {
        int16_t temp_raw = (int16_t)((temp_buf[1] << 8) | temp_buf[0]);
        *temp_c = 25.0f + temp_raw / 256.0f;
    } else {
        *temp_c = 35.0f;
    }
    return ESP_OK;
}

static esp_err_t lis2mdl_init(void) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_write(LIS2MDL_ADDR, 0x60, 0x00));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_write(LIS2MDL_ADDR, 0x60, 0x8c)); // 155Hz, temp enabled
    return ESP_OK;
}

static void transform_mag(vector3f_t *mag) {
    float x = mag->y;
    float y = -mag->x;
    float z = -mag->z;
    mag->x = x * s_calibration.mag_scale - s_calibration.mag_bias.x;
    mag->y = y * s_calibration.mag_scale - s_calibration.mag_bias.y;
    mag->z = z * s_calibration.mag_scale - s_calibration.mag_bias.z;
}

static esp_err_t lis2mdl_read(vector3f_t *mag_ut, float *temp_c) {
    uint8_t buf[6];
    esp_err_t err = i2c_read(LIS2MDL_ADDR, 0x68, buf, sizeof(buf));
    if (err != ESP_OK) {
        return err;
    }
    int16_t raw[3];
    for (int i = 0; i < 3; ++i) {
        raw[i] = (int16_t)((buf[i * 2 + 1] << 8) | buf[i * 2]);
    }
    const float scale = 1.5f; // uT/LSB
    mag_ut->x = raw[0] * scale;
    mag_ut->y = raw[1] * scale;
    mag_ut->z = raw[2] * scale;
    transform_mag(mag_ut);
    uint8_t temp_buf[2];
    if (i2c_read(LIS2MDL_ADDR, 0x6E, temp_buf, sizeof(temp_buf)) == ESP_OK) {
        int16_t temp_raw = (int16_t)((temp_buf[1] << 8) | temp_buf[0]);
        *temp_c = 25.0f + temp_raw / 8.0f;
    } else {
        *temp_c = 35.0f;
    }
    return ESP_OK;
}

static esp_err_t bmp388_read_calib(void) {
    uint8_t buf[21];
    if (i2c_read(BMP388_ADDR, 0x31, buf, sizeof(buf)) != ESP_OK) {
        return ESP_FAIL;
    }
    s_bmp_calib.T1 = (uint16_t)(buf[1] << 8 | buf[0]);
    s_bmp_calib.T2 = (uint16_t)(buf[3] << 8 | buf[2]);
    s_bmp_calib.T3 = (int8_t)buf[4];
    s_bmp_calib.P1 = (int16_t)(buf[6] << 8 | buf[5]);
    s_bmp_calib.P2 = (int16_t)(buf[8] << 8 | buf[7]);
    s_bmp_calib.P3 = (int8_t)buf[9];
    s_bmp_calib.P4 = (int8_t)buf[10];
    s_bmp_calib.P5 = (int16_t)(buf[12] << 8 | buf[11]);
    s_bmp_calib.P6 = (int16_t)(buf[14] << 8 | buf[13]);
    s_bmp_calib.P7 = (int8_t)buf[15];
    s_bmp_calib.P8 = (int8_t)buf[16];
    s_bmp_calib.P9 = (int16_t)(buf[18] << 8 | buf[17]);
    s_bmp_calib.P10 = (int8_t)buf[19];
    s_bmp_calib.P11 = (int8_t)buf[20];
    return ESP_OK;
}

static esp_err_t bmp388_init(void) {
    ESP_RETURN_ON_ERROR(i2c_write(BMP388_ADDR, 0x1B, 0x33), TAG, "baro soft reset");
    vTaskDelay(pdMS_TO_TICKS(20));
    bmp388_read_calib();
    ESP_RETURN_ON_ERROR(i2c_write(BMP388_ADDR, 0x1F, 0x04), TAG, "baro P oversampling");
    ESP_RETURN_ON_ERROR(i2c_write(BMP388_ADDR, 0x1E, 0x03), TAG, "baro T oversampling");
    ESP_RETURN_ON_ERROR(i2c_write(BMP388_ADDR, 0x1D, 0x30), TAG, "baro config");
    ESP_RETURN_ON_ERROR(i2c_write(BMP388_ADDR, 0x1B, 0x30), TAG, "baro normal mode");
    return ESP_OK;
}

static float bmp388_comp_temp(int32_t adc_t) {
    float partial_data1 = (float)(adc_t - s_bmp_calib.T1);
    float partial_data2 = partial_data1 * s_bmp_calib.T2;
    float partial_data3 = partial_data1 * partial_data1;
    float partial_data4 = partial_data3 * s_bmp_calib.T3;
    s_bmp_t_lin = partial_data2 + partial_data4;
    return s_bmp_t_lin;
}

static float bmp388_comp_press(int32_t adc_p) {
    float partial_data1 = s_bmp_t_lin * s_bmp_t_lin;
    float partial_data2 = partial_data1 * s_bmp_calib.P6;
    float partial_data3 = s_bmp_t_lin * s_bmp_calib.P5;
    float partial_data4 = s_bmp_calib.P4;
    float partial_out1 = s_bmp_calib.P3 * s_bmp_t_lin + s_bmp_calib.P2 * s_bmp_t_lin + s_bmp_calib.P1;
    float partial_out2 = ((partial_data2 + partial_data3 + partial_data4) * adc_p) +
                         ((float)s_bmp_calib.P9 * partial_data1 * adc_p * adc_p) +
                         ((float)s_bmp_calib.P10 * partial_data1 * adc_p) + (float)s_bmp_calib.P11 * adc_p * adc_p * adc_p;
    return partial_out1 + partial_out2;
}

static esp_err_t bmp388_read(barometer_state_t *baro) {
    uint8_t buf[6];
    if (i2c_read(BMP388_ADDR, 0x04, buf, sizeof(buf)) != ESP_OK) {
        return ESP_FAIL;
    }
    int32_t adc_p = (int32_t)((uint32_t)buf[2] << 16 | (uint32_t)buf[1] << 8 | buf[0]);
    int32_t adc_t = (int32_t)((uint32_t)buf[5] << 16 | (uint32_t)buf[4] << 8 | buf[3]);
    float temp_c = bmp388_comp_temp(adc_t) / 5120.0f;
    float pressure_pa = bmp388_comp_press(adc_p);
    baro->pressure_hpa = pressure_pa / 100.0f;
    baro->temperature.temperature_c = temp_c;
    baro->altitude_m = 44330.0f * (1.0f - powf(baro->pressure_hpa / 1013.25f, 0.1903f));
    return ESP_OK;
}

static void read_power(power_state_t *power) {
    if (!s_adc_handle) {
        power->battery_voltage_v = 3.7f;
        power->battery_percent = 50;
        power->charging = false;
        return;
    }
    power->battery_voltage_v = 3.7f;
    power->battery_percent = 50;
    int raw = 0;
    if (adc_oneshot_read(s_adc_handle, CONFIG_BATTERY_ADC_CHANNEL, &raw) == ESP_OK) {
        int voltage_mv = 0;
        if (s_adc_cali && adc_cali_raw_to_voltage(s_adc_cali, raw, &voltage_mv) == ESP_OK) {
            power->battery_voltage_v = voltage_mv / 1000.0f;
        } else {
            power->battery_voltage_v = ((float)raw / 4095.0f) * 3.9f;
        }
    }
    float pct = (power->battery_voltage_v - 3.3f) * 100.0f;
    if (pct < 0.0f) {
        pct = 0.0f;
    }
    if (pct > 100.0f) {
        pct = 100.0f;
    }
    power->battery_percent = (uint8_t)pct;
    power->charging = gpio_get_level(CONFIG_BATTERY_CHARGE_GPIO) == 0;
}

void sensors_init(void) {
    memset(&s_state, 0, sizeof(s_state));
    s_lock = xSemaphoreCreateMutexStatic(&s_lock_buffer);
    gpio_config_t charge = {
        .pin_bit_mask = BIT64(CONFIG_BATTERY_CHARGE_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&charge);
    init_i2c();
    init_adc();
    load_calibration();
    lsm6dsr_init();
    lis2mdl_init();
    bmp388_init();
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

    if (lsm6dsr_read(&s_state.imu.linear_accel_g, &s_state.imu.gyro_dps,
                     &s_state.imu.temperature.temperature_c) != ESP_OK) {
        ESP_LOGW(TAG, "IMU read failed");
    }
    s_state.imu.gravity_g.x = 0.0f;
    s_state.imu.gravity_g.y = 0.0f;
    s_state.imu.gravity_g.z = 1.0f;

    if (lis2mdl_read(&s_state.mag.magnetic_ut, &s_state.mag.temperature.temperature_c) != ESP_OK) {
        ESP_LOGW(TAG, "MAG read failed");
    }
    if (bmp388_read(&s_state.baro) != ESP_OK) {
        ESP_LOGW(TAG, "BARO read failed");
    }
    read_power(&s_state.power);

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

