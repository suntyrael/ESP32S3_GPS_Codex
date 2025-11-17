#pragma once

#include <stdbool.h>
#include <time.h>

#include "sensors.h"

typedef enum {
    GPX_LOGGER_STATE_IDLE,
    GPX_LOGGER_STATE_RECORDING,
} gpx_logger_state_t;

typedef struct {
    time_t timestamp_utc;
    uint8_t battery_percent;
    float battery_voltage_v;
    float ride_distance_km;
    float track_distance_km;
    float pbox_elapsed_s;
    char mode_label[16];
    char context_label[32];
} gpx_sample_metadata_t;

void gpx_logger_init(void);
void gpx_logger_start(void);
void gpx_logger_stop(void);
void gpx_logger_push_sample(const sensors_state_t *state, const gpx_sample_metadata_t *meta);
gpx_logger_state_t gpx_logger_get_state(void);

