#include "ui_bike_computer.h"

#include <stdio.h>

static lv_obj_t *create_row(lv_obj_t *parent, const char *label_text) {
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_set_size(row, UI_SCREEN_WIDTH, 45);
    lv_obj_set_style_pad_all(row, 5, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_t *label = lv_label_create(row);
    lv_label_set_text(label, label_text);
    lv_obj_set_style_text_font(label, UI_FONT_MEDIUM, 0);
    lv_obj_t *value = lv_label_create(row);
    lv_obj_set_flex_grow(value, 1);
    lv_obj_set_style_text_font(value, UI_FONT_LARGE, 0);
    lv_label_set_text(value, "0");
    return row;
}

lv_obj_t *ui_bike_computer_create(lv_obj_t *parent) {
    lv_obj_t *screen = lv_obj_create(parent);
    ui_apply_theme(screen);
    lv_obj_set_size(screen, UI_SCREEN_WIDTH, UI_SCREEN_HEIGHT - UI_STATUS_BAR_HEIGHT);
    lv_obj_set_style_pad_all(screen, 0, 0);
    lv_obj_set_flex_flow(screen, LV_FLEX_FLOW_COLUMN);
    lv_obj_t *speed = lv_label_create(screen);
    lv_obj_set_height(speed, 120);
    lv_obj_set_style_text_font(speed, &lv_font_montserrat_48, 0);
    lv_label_set_text(speed, "00.0 km/h");
    create_row(screen, "高度");
    create_row(screen, "距离");
    create_row(screen, "时间");
    create_row(screen, "记录");
    return screen;
}

void ui_bike_computer_update(lv_obj_t *screen, const ui_telemetry_t *telemetry,
                             float ride_distance_km, uint32_t ride_time_s) {
    if (!screen || !telemetry) {
        return;
    }
    char buffer[32];
    lv_obj_t *speed = lv_obj_get_child(screen, 0);
    snprintf(buffer, sizeof(buffer), "%05.1f km/h", telemetry->sensors.gnss.speed_kmh);
    lv_label_set_text(speed, buffer);
    lv_obj_t *height_row = lv_obj_get_child(screen, 1);
    lv_obj_t *height_value = lv_obj_get_child(height_row, 1);
    snprintf(buffer, sizeof(buffer), "%.1fm", telemetry->sensors.baro.altitude_m);
    lv_label_set_text(height_value, buffer);
    lv_obj_t *dist_row = lv_obj_get_child(screen, 2);
    lv_obj_t *dist_value = lv_obj_get_child(dist_row, 1);
    snprintf(buffer, sizeof(buffer), "%.2fkm", ride_distance_km);
    lv_label_set_text(dist_value, buffer);
    lv_obj_t *time_row = lv_obj_get_child(screen, 3);
    lv_obj_t *time_value = lv_obj_get_child(time_row, 1);
    uint32_t hours = ride_time_s / 3600;
    uint32_t minutes = (ride_time_s % 3600) / 60;
    uint32_t seconds = ride_time_s % 60;
    snprintf(buffer, sizeof(buffer), "%02u:%02u:%02u", hours, minutes, seconds);
    lv_label_set_text(time_value, buffer);
    lv_obj_t *rec_row = lv_obj_get_child(screen, 4);
    lv_obj_t *rec_value = lv_obj_get_child(rec_row, 1);
    lv_label_set_text(rec_value, telemetry->logger_state == GPX_LOGGER_STATE_RECORDING ? "REC" : "IDLE");
}

