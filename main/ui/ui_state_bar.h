#pragma once

#include "ui_common.h"

lv_obj_t *ui_state_bar_create(lv_obj_t *parent);
void ui_state_bar_update(lv_obj_t *bar, const ui_telemetry_t *telemetry);

