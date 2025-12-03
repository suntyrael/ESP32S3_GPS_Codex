#ifndef COMPONENT_POWER_H
#define COMPONENT_POWER_H

/**
 * @file power.h
 * @brief 电源管理接口，涵盖 GPIO、PWM 以及外设上电控制。
 */

#include "esp_err.h"
#include <stdbool.h>

typedef struct {
    int sdio_power_pin; /**< SDIO 供电控制引脚。 */
    int sensor_power_pin; /**< 传感器电源控制引脚。 */
    int backlight_pwm_pin; /**< 背光 PWM 引脚。 */
} power_config_t;

/**
 * @brief 初始化电源管理。
 */
esp_err_t power_init(const power_config_t *config);

/**
 * @brief 控制背光亮度。
 */
esp_err_t power_set_backlight(uint8_t duty_percent);

#endif // COMPONENT_POWER_H
