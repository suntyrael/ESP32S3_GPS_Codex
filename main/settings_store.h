/*
 * settings_store.h - 系统设置与传感器校准参数 NVS Flash 持久化存储
 * 命名空间：
 *   - "settings": 系统配置项（功能模式、主题、刷新率、星座、亮度、休眠、RTC、高度校准）
 *   - "cal": 传感器校准数据（陀螺零漂、加速度偏置、磁力计硬铁/软铁矩阵）
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define SETTINGS_STORE_COUNT 8

/**
 * @brief 初始化并从 NVS 加载所有系统配置，若首次启动无记录则使用默认值
 */
esp_err_t settings_store_init(void);

/** @brief 获取配置项当前值 */
int settings_store_get(int idx);

/** @brief 修改配置项值并立即落盘保存到 NVS Flash */
void settings_store_set(int idx, int val);

/** @brief 将所有当前配置项全量保存到 NVS Flash */
esp_err_t settings_store_save_all(void);

/* ==================== 传感器校准参数持久化 ==================== */
typedef struct {
    float gyro_bias_mdps[3];
    float acc_bias_mg[3];
    bool imu_calibrated;
    float mag_bias_mgauss[3];
    float mag_scale[3];
    bool mag_calibrated;
} sensor_calib_data_t;

/** @brief 从 NVS 加载传感器校准数据 */
esp_err_t calib_store_load(sensor_calib_data_t *out);

/** @brief 保存传感器校准数据到 NVS */
esp_err_t calib_store_save(const sensor_calib_data_t *in);
