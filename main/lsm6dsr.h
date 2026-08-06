/*
 * lsm6dsr.h - LSM6DSR IMU 驱动（I2C，地址 0x6A/0x6B 探测）
 * 芯片 ID 宽松校验：WHO_AM_I ∈ {0x6B, 0x6A}
 */
#pragma once

#include "driver/i2c_master.h"

typedef struct lsm6dsr_dev *lsm6dsr_handle_t;

typedef struct {
    float accel_mg[3];      /* 线性加速度，单位 mg */
    float gyro_mdps[3];     /* 角速度，单位 mdps */
    float temp_c;           /* 芯片温度，单位 ℃ */
} lsm6dsr_data_t;

/**
 * @brief 初始化：地址探测 -> WHO_AM_I 校验 -> 软复位 -> 配置（104Hz）
 * @param bus I2C 总线句柄
 * @param[out] out 返回设备句柄（失败时置 NULL）
 * @return ESP_OK 成功
 */
esp_err_t lsm6dsr_init(i2c_master_bus_handle_t bus, lsm6dsr_handle_t *out);

/** @brief 读加速度/陀螺/温度，失败返回错误码 */
esp_err_t lsm6dsr_read(lsm6dsr_handle_t dev, lsm6dsr_data_t *data);
