/*
 * bmp388.h - BMP388 气压计驱动（I2C，地址 0x76/0x77 探测）
 * 整数补偿公式移植自 Bosch 官方 bmp3.c（已验证实现）
 */
#pragma once

#include "driver/i2c_master.h"

typedef struct bmp388_dev *bmp388_handle_t;

typedef struct {
    float temp_c;           /* 温度，℃ */
    float pressure_hpa;     /* 气压，hPa */
    float altitude_m;       /* 海拔，m（标准大气 1013.25hPa） */
} bmp388_data_t;

/**
 * @brief 初始化：地址探测 -> CHIP_ID 校验 -> 校准参数读取 -> 正常模式
 * @param bus I2C 总线句柄
 * @param[out] out 返回设备句柄（失败置 NULL）
 * @return ESP_OK 成功
 */
esp_err_t bmp388_init(i2c_master_bus_handle_t bus, bmp388_handle_t *out);

/** @brief 读温度/气压/海拔，失败返回错误码 */
esp_err_t bmp388_read(bmp388_handle_t dev, bmp388_data_t *data);
