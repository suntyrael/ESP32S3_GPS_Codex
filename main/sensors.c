#include "sensors.h"
#include "config.h"
#include "i2c_bus.h"
#include "lsm6dsr.h"
#include "lis2mdl.h"
#include "bmp388.h"
#include "battery.h"
#include "settings_store.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "sensors";

static SemaphoreHandle_t s_mutex = NULL;
static sensors_state_t s_state = { 0 };
static lsm6dsr_handle_t s_imu = NULL;
static lis2mdl_handle_t s_mag = NULL;
static bmp388_handle_t s_baro = NULL;

/* 真实传感器校准参数缓存（开机从 NVS 加载） */
static sensor_calib_data_t s_calib_data = {
    .mag_scale = { 1.0f, 1.0f, 1.0f }
};

/* IMU 校准过程临时统计：目标 30 帧静止有效采样 */
#define IMU_CALIB_TARGET_COUNT  30
static int s_imu_cal_count = 0;
static float s_imu_cal_sum_gyro[3] = { 0 };
static float s_imu_cal_sum_acc[3] = { 0 };

/* 地磁 8 字校准三维点云极值与八象限覆盖度追踪 */
static float s_mag_min[3] = { 99999.0f, 99999.0f, 99999.0f };
static float s_mag_max[3] = { -99999.0f, -99999.0f, -99999.0f };
static uint8_t s_mag_octant_mask = 0;
static int s_mag_cal_samples = 0;

/* 重力向量低通估计（仅 sensor task 调用）：lin = acc - grav */
#define GRAV_LP_ALPHA   0.10f        /* 20Hz 采样下时间常数 ~0.5s，兼顾响应与稳定 */
static float s_grav_mg[3] = { 0, 0, 0 };
static bool s_grav_init = false;

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

    /* 从 NVS 加载已保存的真实传感器校准参数 */
    calib_store_load(&s_calib_data);

    ESP_LOGI(TAG, "sensors_init 完成: IMU=%s MAG=%s BARO=%s BAT=%s",
             s_state.imu.valid ? "OK" : "FAIL",
             s_state.mag.valid ? "OK" : "FAIL",
             s_state.baro.valid ? "OK" : "FAIL",
             s_state.battery.valid ? "OK" : "FAIL");
    return ESP_OK;
}

esp_err_t sensors_update(void)
{
    /* 继承上一帧完整状态：单通道偶发失败时保留旧数据，仅连续失败降级（D-02） */
    sensors_state_t st = s_state;

    if (s_imu != NULL) {
        lsm6dsr_data_t d;
        if (lsm6dsr_read(s_imu, &d) == ESP_OK) {
            for (int i = 0; i < 3; i++) {
                float acc = d.accel_mg[i];
                float gyro = d.gyro_mdps[i];
                /* 真实应用已校准的零偏偏置 */
                if (s_calib_data.imu_calibrated) {
                    gyro -= s_calib_data.gyro_bias_mdps[i];
                    acc -= s_calib_data.acc_bias_mg[i];
                }
                st.imu.accel_mg[i] = acc;
                /* 重力低通估计 + 线性加速度分离 */
                if (!s_grav_init) {
                    s_grav_mg[i] = acc;
                    s_grav_init = true;
                } else {
                    s_grav_mg[i] += GRAV_LP_ALPHA * (acc - s_grav_mg[i]);
                }
                st.imu.lin_mg[i] = acc - s_grav_mg[i];
                st.imu.gyro_mdps[i] = gyro;
            }
            st.imu.temp_c = d.temp_c;
            st.imu.fails = 0;
            st.imu.valid = true;
        } else if (++st.imu.fails >= SENSOR_FAIL_LIMIT) {
            st.imu.valid = false;
        }
    }
    if (s_mag != NULL) {
        lis2mdl_data_t d;
        if (lis2mdl_read(s_mag, &d) == ESP_OK) {
            for (int i = 0; i < 3; i++) {
                float m = d.mag_mgauss[i];
                /* 真实应用地磁硬铁偏置与软铁矩阵缩放 */
                if (s_calib_data.mag_calibrated) {
                    m = (m - s_calib_data.mag_bias_mgauss[i]) * s_calib_data.mag_scale[i];
                }
                st.mag.mag_mgauss[i] = m;
            }
            st.mag.temp_c = d.temp_c;
            st.mag.fails = 0;
            st.mag.valid = true;
        } else if (++st.mag.fails >= SENSOR_FAIL_LIMIT) {
            st.mag.valid = false;
        }
    }
    if (s_baro != NULL) {
        bmp388_data_t d;
        if (bmp388_read(s_baro, &d) == ESP_OK) {
            st.baro.temp_c = d.temp_c;
            st.baro.pressure_hpa = d.pressure_hpa;
            st.baro.altitude_m = d.altitude_m;
            st.baro.fails = 0;
            st.baro.valid = true;
        } else if (++st.baro.fails >= SENSOR_FAIL_LIMIT) {
            st.baro.valid = false;
        }
    }
    if (s_state.battery.valid) {
        battery_data_t d;
        if (battery_read(&d) == ESP_OK) {
            st.battery.voltage_v = d.voltage_v;
            st.battery.adc_mv = d.adc_mv;
            st.battery.raw_count = d.raw_count;
            st.battery.percent = d.percent;
            st.battery.saturated = d.saturated;
            st.battery.charging = d.charging;
            st.battery.fails = 0;
            st.battery.valid = true;
        } else if (++st.battery.fails >= SENSOR_FAIL_LIMIT) {
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
    if (s_mutex == NULL) {          /* 未初始化防御 */
        memset(out, 0, sizeof(*out));
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

void sensors_calibrate_altitude(float known_alt_m)
{
    if (s_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (s_baro != NULL) {
            bmp388_calibrate_altitude(s_baro, known_alt_m);
        }
        xSemaphoreGive(s_mutex);
    }
}

/* ==================== 真实传感器校准算法实现 ==================== */
void sensors_calibration_reset(void)
{
    s_imu_cal_count = 0;
    memset(s_imu_cal_sum_gyro, 0, sizeof(s_imu_cal_sum_gyro));
    memset(s_imu_cal_sum_acc, 0, sizeof(s_imu_cal_sum_acc));

    for (int i = 0; i < 3; i++) {
        s_mag_min[i] = 99999.0f;
        s_mag_max[i] = -99999.0f;
    }
    s_mag_octant_mask = 0;
    s_mag_cal_samples = 0;
}

bool sensors_calibration_step_imu(int *out_pct, bool *out_is_still)
{
    if (out_pct == NULL || out_is_still == NULL || s_imu == NULL) {
        return false;
    }

    lsm6dsr_data_t d;
    if (lsm6dsr_read(s_imu, &d) != ESP_OK) {
        *out_is_still = false;
        *out_pct = s_imu_cal_count * 100 / IMU_CALIB_TARGET_COUNT;
        return false;
    }

    /* 真实水平静止判定：
     * 1. 陀螺仪各轴绝对值 < 12000 mdps (12 dps)
     * 2. 加速度矢量模长接近 1G (800mg ~ 1200mg) */
    float g_norm = sqrtf(d.accel_mg[0]*d.accel_mg[0] + d.accel_mg[1]*d.accel_mg[1] + d.accel_mg[2]*d.accel_mg[2]);
    bool is_still = (fabsf(d.gyro_mdps[0]) < 12000.0f &&
                     fabsf(d.gyro_mdps[1]) < 12000.0f &&
                     fabsf(d.gyro_mdps[2]) < 12000.0f &&
                     fabsf(g_norm - 1000.0f) < 200.0f);

    *out_is_still = is_still;
    if (!is_still) {
        /* 如果晃动，进度条绝对停止，不采集污染数据 */
        *out_pct = s_imu_cal_count * 100 / IMU_CALIB_TARGET_COUNT;
        return false;
    }

    /* 真实静止：累加计算零偏 */
    for (int i = 0; i < 3; i++) {
        s_imu_cal_sum_gyro[i] += d.gyro_mdps[i];
    }
    s_imu_cal_sum_acc[0] += d.accel_mg[0];
    s_imu_cal_sum_acc[1] += d.accel_mg[1];
    s_imu_cal_sum_acc[2] += (d.accel_mg[2] > 0 ? (d.accel_mg[2] - 1000.0f) : (d.accel_mg[2] + 1000.0f));

    s_imu_cal_count++;
    *out_pct = s_imu_cal_count * 100 / IMU_CALIB_TARGET_COUNT;

    if (s_imu_cal_count >= IMU_CALIB_TARGET_COUNT) {
        for (int i = 0; i < 3; i++) {
            s_calib_data.gyro_bias_mdps[i] = s_imu_cal_sum_gyro[i] / (float)IMU_CALIB_TARGET_COUNT;
            s_calib_data.acc_bias_mg[i] = s_imu_cal_sum_acc[i] / (float)IMU_CALIB_TARGET_COUNT;
        }
        s_calib_data.imu_calibrated = true;
        *out_pct = 100;
        return true;
    }
    return false;
}

bool sensors_calibration_step_mag(int *out_pct, bool *out_motion_ok)
{
    if (out_pct == NULL || out_motion_ok == NULL || s_mag == NULL) {
        return false;
    }

    lis2mdl_data_t d;
    if (lis2mdl_read(s_mag, &d) != ESP_OK) {
        *out_motion_ok = false;
        *out_pct = 0;
        return false;
    }

    /* 动态追踪三维磁场极值 */
    for (int i = 0; i < 3; i++) {
        if (d.mag_mgauss[i] < s_mag_min[i]) s_mag_min[i] = d.mag_mgauss[i];
        if (d.mag_mgauss[i] > s_mag_max[i]) s_mag_max[i] = d.mag_mgauss[i];
    }
    s_mag_cal_samples++;

    float span_x = s_mag_max[0] - s_mag_min[0];
    float span_y = s_mag_max[1] - s_mag_min[1];
    float span_z = s_mag_max[2] - s_mag_min[2];

    /* 关键：如果设备静止在桌上，三轴极差几乎为 0 (< 60 mGauss = 6uT)
     * 判定为未在做 8 字晃动，进度条绝对为 0%！ */
    if (span_x < 60.0f && span_y < 60.0f && span_z < 60.0f) {
        *out_motion_ok = false;
        *out_pct = 0;
        return false;
    }

    *out_motion_ok = true;

    /* 统计 8 象限空间覆盖度 */
    float cx = (s_mag_max[0] + s_mag_min[0]) / 2.0f;
    float cy = (s_mag_max[1] + s_mag_min[1]) / 2.0f;
    float cz = (s_mag_max[2] + s_mag_min[2]) / 2.0f;

    int octant = 0;
    if (d.mag_mgauss[0] > cx) octant |= 1;
    if (d.mag_mgauss[1] > cy) octant |= 2;
    if (d.mag_mgauss[2] > cz) octant |= 4;
    s_mag_octant_mask |= (1 << octant);

    int oct_count = 0;
    for (int i = 0; i < 8; i++) {
        if (s_mag_octant_mask & (1 << i)) {
            oct_count++;
        }
    }

    /* 真实空间覆盖进度：象限覆盖率贡献 60% + 三轴空间跨度贡献 40% */
    int oct_pct = oct_count * 60 / 8;
    float span_req = 280.0f; /* 目标跨度 280 mGauss = 28 uT */
    float span_score = (fminf(span_x, span_req) + fminf(span_y, span_req) + fminf(span_z, span_req)) / (3.0f * span_req);
    int span_pct = (int)(span_score * 40.0f);

    int total_pct = oct_pct + span_pct;
    if (total_pct > 100) total_pct = 100;
    *out_pct = total_pct;

    /* 严格判定：只有覆盖 >= 7 个三维象限且动态范围充足时才判定 8 字校准完成 */
    if (total_pct >= 100 && oct_count >= 7 && s_mag_cal_samples >= 40) {
        /* 计算真实硬铁偏置 */
        for (int i = 0; i < 3; i++) {
            s_calib_data.mag_bias_mgauss[i] = (s_mag_max[i] + s_mag_min[i]) / 2.0f;
        }
        /* 计算软铁比例因子 */
        float rx = (s_mag_max[0] - s_mag_min[0]) / 2.0f;
        float ry = (s_mag_max[1] - s_mag_min[1]) / 2.0f;
        float rz = (s_mag_max[2] - s_mag_min[2]) / 2.0f;
        if (rx > 10.0f && ry > 10.0f && rz > 10.0f) {
            float ravg = (rx + ry + rz) / 3.0f;
            s_calib_data.mag_scale[0] = ravg / rx;
            s_calib_data.mag_scale[1] = ravg / ry;
            s_calib_data.mag_scale[2] = ravg / rz;
        } else {
            s_calib_data.mag_scale[0] = 1.0f;
            s_calib_data.mag_scale[1] = 1.0f;
            s_calib_data.mag_scale[2] = 1.0f;
        }
        s_calib_data.mag_calibrated = true;
        *out_pct = 100;
        return true;
    }
    return false;
}

void sensors_calibration_save(void)
{
    calib_store_save(&s_calib_data);
    ESP_LOGI(TAG, "Sensor calibration saved to NVS and active in runtime");
}
