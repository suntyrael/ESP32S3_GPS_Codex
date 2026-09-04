#include "sensors.h"
#include "config.h"
#include "i2c_bus.h"
#include "lsm6dsr.h"
#include "lis2mdl.h"
#include "bmp388.h"
#include "battery.h"
#include "settings_store.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

/* 真实校准引擎运行态 */
static sensors_calib_mode_t s_calib_mode = SENSORS_CALIB_MODE_IDLE;
static sensors_calib_live_status_t s_live_calib_status = { 0 };

/* IMU 校准过程临时统计：目标 30 帧静止有效采样 */
#define IMU_CALIB_TARGET_COUNT  30
static int s_imu_cal_count = 0;
static float s_imu_cal_sum_gyro[3] = { 0 };
static float s_imu_cal_sum_acc[3] = { 0 };

/* 地磁 8 字校准三维点云极值与八象限覆盖度追踪，30 秒超时机制 */
#define MAG_CALIB_TIMEOUT_MS    30000
static uint32_t s_mag_cal_start_ms = 0;
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

            /* 处于 IMU 真实校准模式时，执行严格物理静止采样与抗振动检测 */
            if (s_calib_mode == SENSORS_CALIB_MODE_IMU) {
                float g_norm = sqrtf(d.accel_mg[0]*d.accel_mg[0] + d.accel_mg[1]*d.accel_mg[1] + d.accel_mg[2]*d.accel_mg[2]);
                /* 严苛抗振动静止判定：
                 * 1. 陀螺仪角速度绝对值严格 < 3500 mdps (3.5 dps)
                 * 2. 加速度合力模长严格限制在 940mg ~ 1060mg（1G 静态公差仅 60mg）
                 * 只要有任何手持晃动或桌面敲击振动，合力跳变必然导致 is_still 为 false */
                bool is_still = (fabsf(d.gyro_mdps[0]) < 3500.0f &&
                                 fabsf(d.gyro_mdps[1]) < 3500.0f &&
                                 fabsf(d.gyro_mdps[2]) < 3500.0f &&
                                 fabsf(g_norm - 1000.0f) < 60.0f);
                s_live_calib_status.imu_is_still = is_still;
                if (!is_still) {
                    /* 关键抗振动惩罚：一旦检测到振动或晃动，立即清空累积样本，进度直接归零！
                     * 确保必须在台面上连续、纯净静止平放 2 秒（40 帧）才能完成校准，振动时绝对不可能涨进度！ */
                    s_imu_cal_count = 0;
                    memset(s_imu_cal_sum_gyro, 0, sizeof(s_imu_cal_sum_gyro));
                    memset(s_imu_cal_sum_acc, 0, sizeof(s_imu_cal_sum_acc));
                    s_live_calib_status.imu_pct = 0;
                } else {
                    for (int i = 0; i < 3; i++) {
                        s_imu_cal_sum_gyro[i] += d.gyro_mdps[i];
                    }
                    s_imu_cal_sum_acc[0] += d.accel_mg[0];
                    s_imu_cal_sum_acc[1] += d.accel_mg[1];
                    s_imu_cal_sum_acc[2] += (d.accel_mg[2] > 0 ? (d.accel_mg[2] - 1000.0f) : (d.accel_mg[2] + 1000.0f));
                    s_imu_cal_count++;
                    if (s_imu_cal_count >= IMU_CALIB_TARGET_COUNT) {
                        for (int i = 0; i < 3; i++) {
                            s_calib_data.gyro_bias_mdps[i] = s_imu_cal_sum_gyro[i] / (float)IMU_CALIB_TARGET_COUNT;
                            s_calib_data.acc_bias_mg[i] = s_imu_cal_sum_acc[i] / (float)IMU_CALIB_TARGET_COUNT;
                        }
                        s_calib_data.imu_calibrated = true;
                        s_live_calib_status.imu_ready = true;
                        s_live_calib_status.imu_pct = 100;
                    } else {
                        s_live_calib_status.imu_pct = s_imu_cal_count * 100 / IMU_CALIB_TARGET_COUNT;
                    }
                }
            }
        } else if (++st.imu.fails >= SENSOR_FAIL_LIMIT) {
            st.imu.valid = false;
        }
    }
    if (s_mag != NULL) {
        lis2mdl_data_t d;
        if (lis2mdl_read(s_mag, &d) == ESP_OK) {
            float cal_m[3];
            for (int i = 0; i < 3; i++) {
                float m = d.mag_mgauss[i];
                /* 真实应用地磁硬铁偏置与软铁矩阵缩放（芯片物理通道空间） */
                if (s_calib_data.mag_calibrated) {
                    m = (m - s_calib_data.mag_bias_mgauss[i]) * s_calib_data.mag_scale[i];
                }
                cal_m[i] = m;
            }
            /* 统一映射至机身设备坐标系（config.h 参数化：X=机身向右，Y=机身向前，Z=屏幕法线向上） */
            st.mag.mag_mgauss[0] = cal_m[MAG_AXIS_X_SRC] * (float)MAG_AXIS_X_SIGN;
            st.mag.mag_mgauss[1] = cal_m[MAG_AXIS_Y_SRC] * (float)MAG_AXIS_Y_SIGN;
            st.mag.mag_mgauss[2] = cal_m[MAG_AXIS_Z_SRC] * (float)MAG_AXIS_Z_SIGN;

            st.mag.temp_c = d.temp_c;
            st.mag.fails = 0;
            st.mag.valid = true;

            /* 处于地磁真实 8 字校准模式时，执行三维点云极差与八象限覆盖度分析 */
            if (s_calib_mode == SENSORS_CALIB_MODE_MAG) {
                uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
                if (now_ms - s_mag_cal_start_ms >= MAG_CALIB_TIMEOUT_MS) {
                    /* 30 秒超时未完成：判定超时失败，需要重新校准！ */
                    s_live_calib_status.timeout = true;
                    s_live_calib_status.mag_ready = false;
                } else {
                    for (int i = 0; i < 3; i++) {
                        if (d.mag_mgauss[i] < s_mag_min[i]) s_mag_min[i] = d.mag_mgauss[i];
                        if (d.mag_mgauss[i] > s_mag_max[i]) s_mag_max[i] = d.mag_mgauss[i];
                    }
                    s_mag_cal_samples++;
                    float span_x = s_mag_max[0] - s_mag_min[0];
                    float span_y = s_mag_max[1] - s_mag_min[1];
                    float span_z = s_mag_max[2] - s_mag_min[2];

                    /* 关键防假判定：如果设备静止平放，三轴跨度不足以构成 3D 旋转（任意一轴极差 < 120 mGauss = 12uT），
                     * 绝对判定为静止未做 8 字，进度条恒为 0%！绝对不涨！ */
                    if (span_x < 120.0f || span_y < 120.0f || span_z < 120.0f) {
                        s_live_calib_status.mag_motion_ok = false;
                        s_live_calib_status.mag_pct = 0;
                    } else {
                        s_live_calib_status.mag_motion_ok = true;
                        float cx = (s_mag_max[0] + s_mag_min[0]) / 2.0f;
                        float cy = (s_mag_max[1] + s_mag_min[1]) / 2.0f;
                        float cz = (s_mag_max[2] + s_mag_min[2]) / 2.0f;

                        int octant = 0;
                        if (d.mag_mgauss[0] > cx) octant |= 1;
                        if (d.mag_mgauss[1] > cy) octant |= 2;
                        if (d.mag_mgauss[2] > cz) octant |= 4;
                        s_mag_octant_mask |= (1 << octant);

                        int oct_cnt = 0;
                        for (int i = 0; i < 8; i++) {
                            if (s_mag_octant_mask & (1 << i)) oct_cnt++;
                        }

                        if (oct_cnt < 2) {
                            /* 象限覆盖不足 2 个，仍然判定为初始静止态，进度 0% */
                            s_live_calib_status.mag_motion_ok = false;
                            s_live_calib_status.mag_pct = 0;
                        } else {
                            /* 真实空间覆盖进度：象限覆盖贡献 60% + 三轴空间跨度贡献 40% */
                            int oct_pct = oct_cnt * 60 / 8;
                            float span_req = 320.0f; /* 目标跨度 320 mGauss = 32 uT */
                            float span_score = (fminf(span_x, span_req) + fminf(span_y, span_req) + fminf(span_z, span_req)) / (3.0f * span_req);
                            int span_pct = (int)(span_score * 40.0f);
                            int total_pct = oct_pct + span_pct;
                            if (total_pct > 100) total_pct = 100;
                            s_live_calib_status.mag_pct = total_pct;

                            if (total_pct >= 100 && oct_cnt >= 7 && s_mag_cal_samples >= 40) {
                                for (int i = 0; i < 3; i++) {
                                    s_calib_data.mag_bias_mgauss[i] = (s_mag_max[i] + s_mag_min[i]) / 2.0f;
                                }
                                float rx = (s_mag_max[0] - s_mag_min[0]) / 2.0f;
                                float ry = (s_mag_max[1] - s_mag_min[1]) / 2.0f;
                                float rz = (s_mag_max[2] - s_mag_min[2]) / 2.0f;
                                if (rx > 10.0f && ry > 10.0f && rz > 10.0f) {
                                    float ravg = (rx + ry + rz) / 3.0f;
                                    s_calib_data.mag_scale[0] = ravg / rx;
                                    s_calib_data.mag_scale[1] = ravg / ry;
                                    s_calib_data.mag_scale[2] = ravg / rz;
                                }
                                s_calib_data.mag_calibrated = true;
                                s_live_calib_status.mag_ready = true;
                                s_live_calib_status.mag_pct = 100;
                            }
                        }
                    }
                }
            }
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

    /* 电子罗盘磁航向解算（结合 IMU 加速度计进行全姿态 3D 空间正交切平面解算） */
    if (st.mag.valid) {
        float mx = st.mag.mag_mgauss[0];
        float my = st.mag.mag_mgauss[1];
        float mz = st.mag.mag_mgauss[2];

        float heading = 0.0f;
        if (st.imu.valid) {
            float ax = st.imu.accel_mg[0];
            float ay = st.imu.accel_mg[1];
            float az = st.imu.accel_mg[2];
            float g_norm = sqrtf(ax * ax + ay * ay + az * az);
            /* 准静态重力矢量区间判定：0.5G ~ 1.5G (500mg ~ 1500mg) */
            if (g_norm >= 500.0f && g_norm <= 1500.0f) {
                float ux = ax / g_norm;
                float uy = ay / g_norm;
                float uz = az / g_norm;

                /* 1. 东向水平正交向量 E = m x u */
                float ex_raw = my * uz - mz * uy;
                float ey_raw = mz * ux - mx * uz;
                float ez_raw = mx * uy - my * ux;
                float e_norm = sqrtf(ex_raw * ex_raw + ey_raw * ey_raw + ez_raw * ez_raw);

                if (e_norm > 1e-4f) {
                    float ex = ex_raw / e_norm;
                    float ey = ey_raw / e_norm;
                    float ez = ez_raw / e_norm;

                    /* 2. 北向水平正交向量 N = u x e */
                    float nx = uy * ez - uz * ey;
                    float ny = uz * ex - ux * ez;
                    float nz = ux * ey - uy * ex;

                    /* 3. 机身自然前向视线向量 f_ref = (0, uz, -uy)：
                     * 平放 (uz=1, uy=0) 对应机身顶部 (0, 1, 0)
                     * 垂直 (uz=0, uy=1) 对应机身背面 (0, 0, -1) 正对用户视线前方
                     * 在平放到垂直放置的整个过渡移动过程中保持数学严格正交且绝对平滑 */
                    float sin_hdg = uz * ey - uy * ez;
                    float cos_hdg = uz * ny - uy * nz;

                    heading = atan2f(sin_hdg, cos_hdg) * (180.0f / (float)M_PI);
                } else {
                    heading = atan2f(-mx, my) * (180.0f / (float)M_PI);
                }
            } else {
                heading = atan2f(-mx, my) * (180.0f / (float)M_PI);
            }
        } else {
            heading = atan2f(-mx, my) * (180.0f / (float)M_PI);
        }

        if (heading < 0.0f) {
            heading += 360.0f;
        }
        if (heading < 0.05f || heading >= 360.0f) {
            heading = 0.0f;
        }
        st.mag.heading_deg = heading;
    } else {
        st.mag.heading_deg = 0.0f;
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

/* ==================== 真实传感器校准接口实现 ==================== */
void sensors_calibration_start(sensors_calib_mode_t mode)
{
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_calib_mode = mode;
        if (mode == SENSORS_CALIB_MODE_IMU) {
            s_imu_cal_count = 0;
            memset(s_imu_cal_sum_gyro, 0, sizeof(s_imu_cal_sum_gyro));
            memset(s_imu_cal_sum_acc, 0, sizeof(s_imu_cal_sum_acc));
            s_live_calib_status.imu_pct = 0;
            s_live_calib_status.imu_is_still = false;
            s_live_calib_status.imu_ready = false;
        } else if (mode == SENSORS_CALIB_MODE_MAG) {
            for (int i = 0; i < 3; i++) {
                s_mag_min[i] = 99999.0f;
                s_mag_max[i] = -99999.0f;
            }
            s_mag_octant_mask = 0;
            s_mag_cal_samples = 0;
            s_mag_cal_start_ms = (uint32_t)(esp_timer_get_time() / 1000);
            s_live_calib_status.mag_pct = 0;
            s_live_calib_status.mag_motion_ok = false;
            s_live_calib_status.mag_ready = false;
            s_live_calib_status.timeout = false;
        }
        s_live_calib_status.mode = mode;
        xSemaphoreGive(s_mutex);
        ESP_LOGI(TAG, "Calibration mode switched -> %d", (int)mode);
    }
}

void sensors_calibration_cancel(void)
{
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_calib_mode = SENSORS_CALIB_MODE_IDLE;
        s_live_calib_status.mode = SENSORS_CALIB_MODE_IDLE;
        s_live_calib_status.imu_pct = 0;
        s_live_calib_status.mag_pct = 0;
        s_live_calib_status.imu_ready = false;
        s_live_calib_status.mag_ready = false;
        xSemaphoreGive(s_mutex);
        ESP_LOGI(TAG, "Calibration cancelled");
    }
}

void sensors_calibration_get_status(sensors_calib_live_status_t *out)
{
    if (out == NULL) return;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        *out = s_live_calib_status;
        xSemaphoreGive(s_mutex);
    }
}

void sensors_calibration_save(void)
{
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_calib_mode = SENSORS_CALIB_MODE_IDLE;
        s_live_calib_status.mode = SENSORS_CALIB_MODE_IDLE;
        calib_store_save(&s_calib_data);
        xSemaphoreGive(s_mutex);
        ESP_LOGI(TAG, "Sensor calibration saved to NVS and active in runtime");
    }
}
