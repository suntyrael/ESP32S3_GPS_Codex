#pragma once

#include <stdbool.h>
#include "sensors.h"

typedef enum {
    GPX_LOGGER_STATE_IDLE,
    GPX_LOGGER_STATE_RECORDING,
} gpx_logger_state_t;

void gpx_logger_init(void);
void gpx_logger_start(void);
void gpx_logger_stop(void);
void gpx_logger_push_sample(const sensors_state_t *state);
gpx_logger_state_t gpx_logger_get_state(void);

