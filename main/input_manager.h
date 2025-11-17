#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"

typedef enum {
    INPUT_EVENT_NONE = 0,
    INPUT_EVENT_ENCODER_LEFT,
    INPUT_EVENT_ENCODER_RIGHT,
    INPUT_EVENT_BUTTON_SHORT,
    INPUT_EVENT_BUTTON_MEDIUM,
    INPUT_EVENT_BUTTON_LONG,
    INPUT_EVENT_BUTTON_DOUBLE
} input_event_type_t;

typedef struct {
    input_event_type_t type;
    int32_t value;
    uint32_t duration_ms;
} input_event_t;

void input_manager_init(void);
bool input_manager_get_event(input_event_t *event_out, TickType_t ticks_to_wait);

