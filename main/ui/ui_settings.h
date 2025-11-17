#pragma once

#include "ui_common.h"

typedef enum {
    SETTINGS_OPTION_IMU_CAL,
    SETTINGS_OPTION_MAG_CAL,
    SETTINGS_OPTION_GNSS_RATE,
    SETTINGS_OPTION_CONSTELLATION,
    SETTINGS_OPTION_BRIGHTNESS,
    SETTINGS_OPTION_ABOUT,
    SETTINGS_OPTION_COUNT
} settings_option_t;

lv_obj_t *ui_settings_create(lv_obj_t *parent);
void ui_settings_update(lv_obj_t *screen, settings_option_t selected, uint8_t gnss_rate_hz);

