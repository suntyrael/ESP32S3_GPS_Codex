#pragma once

#include "ui_common.h"

typedef enum {
    SETTINGS_OPTION_IMU_CAL,
    SETTINGS_OPTION_MAG_CAL,
    SETTINGS_OPTION_GNSS_RATE,
    SETTINGS_OPTION_GNSS_DYNAMIC,
    SETTINGS_OPTION_CONSTELLATION,
    SETTINGS_OPTION_PBOX_THRESHOLD,
    SETTINGS_OPTION_BRIGHTNESS,
    SETTINGS_OPTION_ABOUT,
    SETTINGS_OPTION_COUNT
} settings_option_t;

typedef struct {
    settings_option_t selected;
    uint8_t gnss_rate_hz;
    const char *constellation_label;
    const char *dynamic_label;
    float pbox_threshold_g;
    char imu_status[48];
    char mag_status[48];
} settings_view_model_t;

lv_obj_t *ui_settings_create(lv_obj_t *parent);
void ui_settings_update(lv_obj_t *screen, const settings_view_model_t *model);

