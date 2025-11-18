#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"

/**
 * @brief 输入事件类型枚举
 */
typedef enum {
    INPUT_EVENT_NONE = 0,
    INPUT_EVENT_ENCODER_LEFT,
    INPUT_EVENT_ENCODER_RIGHT,
    INPUT_EVENT_BUTTON_SHORT,
    INPUT_EVENT_BUTTON_MEDIUM,
    INPUT_EVENT_BUTTON_LONG,
    INPUT_EVENT_BUTTON_DOUBLE,
} input_event_type_t;

/**
 * @brief 输入事件结构
 *
 * type: 事件类型
 * value: 对于旋转编码器事件，表示步数（正负代表方向）；按键事件填 0
 * duration_ms: 按键按下持续时间，旋转编码器事件填 0
 */
typedef struct {
    input_event_type_t type;
    int32_t value;
    uint32_t duration_ms;
} input_event_t;

void input_manager_init(void);
bool input_manager_get_event(input_event_t *event_out, TickType_t ticks_to_wait);
