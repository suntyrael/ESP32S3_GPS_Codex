#ifndef COMPONENT_SENSOR_H
#define COMPONENT_SENSOR_H

/**
 * @file sensor.h
 * @brief 传感器采集接口，支持轴向变换与滤波。
 */

#include "esp_err.h"
#include <stdint.h>

typedef struct {
    int i2c_port;       /**< I2C 端口号。 */
    int sda_pin;        /**< SDA 引脚。 */
    int scl_pin;        /**< SCL 引脚。 */
    uint32_t freq_hz;   /**< I2C 频率。 */
} sensor_bus_t;

typedef struct {
    float ax; /**< X 轴加速度 (m/s^2) */
    float ay; /**< Y 轴加速度 (m/s^2) */
    float az; /**< Z 轴加速度 (m/s^2) */
    float gx; /**< X 轴角速度 (dps) */
    float gy; /**< Y 轴角速度 (dps) */
    float gz; /**< Z 轴角速度 (dps) */
    uint64_t timestamp_ms; /**< 时间戳 */
} sensor_frame_t;

/**
 * @brief 初始化传感器采集总线。
 */
esp_err_t sensor_init_bus(const sensor_bus_t *bus);

/**
 * @brief 采集并完成坐标轴变换。
 * @param frame 输出传感器数据。
 */
esp_err_t sensor_read_frame(sensor_frame_t *frame);

#endif // COMPONENT_SENSOR_H
