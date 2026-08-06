/*
 * i2c_bus.h - I2C 总线层（v6 新 master 驱动）
 * 总线句柄由 app_main 创建，注入各驱动模块；总线自带互斥（驱动线程安全）
 */
#pragma once

#include "driver/i2c_master.h"

/**
 * @brief 初始化 I2C 总线（I2C_NUM_0，SCL/SDA 来自 config.h）
 * @param[out] out_handle 返回总线句柄
 * @return ESP_OK 成功
 */
esp_err_t i2c_bus_init(i2c_master_bus_handle_t *out_handle);
