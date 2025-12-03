#ifndef COMPONENT_INPUT_H
#define COMPONENT_INPUT_H

/**
 * @file input.h
 * @brief 按键与编码器输入解析，包含防抖和优先级处理。
 */

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    INPUT_EVENT_NONE = 0,
    INPUT_EVENT_CLICK,
    INPUT_EVENT_LONG_PRESS,
    INPUT_EVENT_ENCODER_INC,
    INPUT_EVENT_ENCODER_DEC,
} input_event_type_t;

typedef struct {
    input_event_type_t type; /**< 输入事件类型。 */
    uint32_t timestamp_ms;   /**< 事件时间戳。 */
    uint8_t priority;        /**< 事件优先级，数值越大优先级越高。 */
} input_event_t;

typedef struct {
    int button_pin;       /**< 按键 GPIO。 */
    int encoder_a_pin;    /**< 编码器 A 相。 */
    int encoder_b_pin;    /**< 编码器 B 相。 */
    uint32_t debounce_ms; /**< 防抖时间。 */
} input_config_t;

/**
 * @brief 初始化输入设备。
 */
esp_err_t input_init(const input_config_t *config);

/**
 * @brief 读取并解析输入事件。
 */
esp_err_t input_poll_event(input_event_t *event);

#endif // COMPONENT_INPUT_H
