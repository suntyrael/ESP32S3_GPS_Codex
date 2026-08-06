/*
 * sensors.h - 汇聚层（唯一写者）
 * 所有硬件状态由 sensor_task 周期调用 sensors_update() 写入，
 * 其他任务通过 sensors_get_state() 获取互斥保护下的按值拷贝快照（D-02）。
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    bool valid;
    float accel_mg[3];      /* 总加速度（含重力）mg */
    float lin_mg[3];        /* 线性加速度（重力估计分离后）mg */
    float gyro_mdps[3];     /* 角速度 mdps */
    float temp_c;
} sensors_imu_t;

typedef struct {
    bool valid;
    float mag_mgauss[3];
    float temp_c;
} sensors_mag_t;

typedef struct {
    bool valid;
    float temp_c;
    float pressure_hpa;
    float altitude_m;
} sensors_baro_t;

typedef struct {
    bool valid;
    float voltage_v;
    uint8_t percent;
    bool saturated;
    bool charging;
} sensors_battery_t;

typedef struct {
    sensors_imu_t imu;
    sensors_mag_t mag;
    sensors_baro_t baro;
    sensors_battery_t battery;
    /* GNSS 字段在阶段 3 扩展 */
} sensors_state_t;

/**
 * @brief 初始化：I2C 总线 + 三个传感器驱动 + 电池
 * @return ESP_OK 全部关键器件初始化成功（单器件失败仍返回 ESP_OK，仅 valid=false）
 */
esp_err_t sensors_init(void);

/** @brief 周期采样并更新内部状态（仅 sensor_task 调用） */
esp_err_t sensors_update(void);

/** @brief 获取状态快照（按值拷贝，线程安全） */
void sensors_get_state(sensors_state_t *out);

/** @brief 各通道 valid 汇总：返回 true 表示全部就绪 */
bool sensors_all_ready(void);
