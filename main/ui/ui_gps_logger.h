#pragma once

#include "ui_common.h"

lv_obj_t *ui_gps_logger_create(lv_obj_t *parent);
void ui_gps_logger_update(lv_obj_t *screen, const ui_telemetry_t *telemetry,
                          float track_distance_km, uint32_t track_time_s);

