#ifndef COMPONENT_RTC_H
#define COMPONENT_RTC_H

/**
 * @file rtc.h
 * @brief RTC 同步与校时接口。
 */

#include "esp_err.h"
#include <stdint.h>

typedef struct {
    int i2c_port; /**< RTC 所在 I2C 端口。 */
    int sda_pin;  /**< SDA 引脚。 */
    int scl_pin;  /**< SCL 引脚。 */
} rtc_config_t;

/**
 * @brief 初始化 RTC 外设。
 */
esp_err_t rtc_init(const rtc_config_t *config);

/**
 * @brief 同步系统时间到 RTC。
 */
esp_err_t rtc_sync_from_gnss(uint64_t timestamp_ms);

#endif // COMPONENT_RTC_H
