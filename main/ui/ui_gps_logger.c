#include "ui_gps_logger.h"

#include <stdio.h>

lv_obj_t *ui_gps_logger_create(lv_obj_t *parent) {
    lv_obj_t *screen = lv_obj_create(parent);
    ui_apply_theme(screen);
    lv_obj_set_size(screen, UI_SCREEN_WIDTH, UI_SCREEN_HEIGHT - UI_STATUS_BAR_HEIGHT);
    lv_obj_set_flex_flow(screen, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(screen, 5, 0);
    lv_obj_t *speed = lv_label_create(screen);
    lv_obj_set_style_text_font(speed, &lv_font_montserrat_32, 0);
    lv_label_set_text(speed, "0.0 km/h");
    lv_obj_t *track = lv_label_create(screen);
    lv_obj_set_size(track, UI_SCREEN_WIDTH - 10, 120);
    lv_label_set_text(track, "轨迹图");
    lv_obj_t *dist = lv_label_create(screen);
    lv_obj_t *time = lv_label_create(screen);
    lv_obj_set_style_text_font(dist, UI_FONT_LARGE, 0);
    lv_obj_set_style_text_font(time, UI_FONT_LARGE, 0);
    return screen;
}

void ui_gps_logger_update(lv_obj_t *screen, const ui_telemetry_t *telemetry,
                          float track_distance_km, uint32_t track_time_s) {
    if (!screen || !telemetry) {
        return;
    }
    char buffer[32];
    lv_obj_t *speed = lv_obj_get_child(screen, 0);
    snprintf(buffer, sizeof(buffer), "%.1f km/h", telemetry->sensors.gnss.speed_kmh);
    lv_label_set_text(speed, buffer);
    lv_obj_t *dist = lv_obj_get_child(screen, 2);
    snprintf(buffer, sizeof(buffer), "距离 %.2f km", track_distance_km);
    lv_label_set_text(dist, buffer);
    lv_obj_t *time = lv_obj_get_child(screen, 3);
    uint32_t minutes = track_time_s / 60;
    uint32_t seconds = track_time_s % 60;
    snprintf(buffer, sizeof(buffer), "时间 %02u:%02u", minutes, seconds);
    lv_label_set_text(time, buffer);
}

