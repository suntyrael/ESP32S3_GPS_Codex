#pragma once

#include "ui_common.h"

lv_obj_t *ui_bike_computer_create(lv_obj_t *parent);
void ui_bike_computer_update(lv_obj_t *screen, const ui_telemetry_t *telemetry,
                             float ride_distance_km, uint32_t ride_time_s);

