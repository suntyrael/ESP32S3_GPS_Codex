#pragma once

#include "ui_common.h"

lv_obj_t *ui_gnss_info_create(lv_obj_t *parent);
void ui_gnss_info_update(lv_obj_t *screen, const ui_telemetry_t *telemetry);

