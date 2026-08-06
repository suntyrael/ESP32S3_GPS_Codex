/*
 * lis2mdl.h - LIS2MDL 磁力计驱动（I2C，固定地址 0x1E）
 * WHO_AM_I = 0x40
 */
#pragma once

#include "driver/i2c_master.h"

typedef struct lis2mdl_dev *lis2mdl_handle_t;

typedef struct {
    float mag_mgauss[3];    /* 磁场，单位 mGauss */
    float temp_c;           /* 芯片温度，单位 ℃ */
} lis2mdl_data_t;

/**
 * @brief 初始化：绑定 0x1E -> WHO_AM_I 校验 -> 连续模式
 * @param bus I2C 总线句柄
 * @param[out] out 返回设备句柄（失败置 NULL）
 * @return ESP_OK 成功
 */
esp_err_t lis2mdl_init(i2c_master_bus_handle_t bus, lis2mdl_handle_t *out);

/** @brief 读磁场/温度，失败返回错误码 */
esp_err_t lis2mdl_read(lis2mdl_handle_t dev, lis2mdl_data_t *data);
