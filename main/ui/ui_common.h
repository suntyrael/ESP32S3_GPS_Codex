#pragma once

#include "lvgl.h"
#include "sensors.h"
#include "gpx_logger.h"

#define UI_SCREEN_WIDTH      240
#define UI_SCREEN_HEIGHT     320
#define UI_STATUS_BAR_HEIGHT 20

#define UI_FONT_SMALL   &lv_font_montserrat_8
#define UI_FONT_MEDIUM  &lv_font_montserrat_12
#define UI_FONT_LARGE   &lv_font_montserrat_16
#define UI_FONT_XLARGE  &lv_font_montserrat_24

typedef struct {
    sensors_state_t sensors;
    gpx_logger_state_t logger_state;
} ui_telemetry_t;

void ui_apply_theme(lv_obj_t *obj);

