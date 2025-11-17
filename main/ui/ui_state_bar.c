#include "ui_state_bar.h"

#include <stdio.h>

static lv_obj_t *create_label(lv_obj_t *parent, const char *text) {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_style_text_font(label, UI_FONT_SMALL, 0);
    lv_label_set_text(label, text);
    return label;
}

lv_obj_t *ui_state_bar_create(lv_obj_t *parent) {
    lv_obj_t *bar = lv_obj_create(parent);
    lv_obj_set_size(bar, UI_SCREEN_WIDTH, UI_STATUS_BAR_HEIGHT);
    lv_obj_set_style_bg_color(bar, lv_color_black(), 0);
    lv_obj_set_style_pad_all(bar, 2, 0);
    lv_obj_set_flex_flow(bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(bar, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    create_label(bar, "GPS");
    create_label(bar, "SAT0");
    create_label(bar, "SD");
    create_label(bar, "BAT");
    create_label(bar, "CHG");
    return bar;
}

void ui_state_bar_update(lv_obj_t *bar, const ui_telemetry_t *telemetry) {
    if (!bar || !telemetry) {
        return;
    }
    char buffer[32];
    lv_obj_t *child = lv_obj_get_child(bar, 1);
    snprintf(buffer, sizeof(buffer), "%02u", telemetry->sensors.gnss.sats_in_use);
    lv_label_set_text(child, buffer);
    child = lv_obj_get_child(bar, 3);
    snprintf(buffer, sizeof(buffer), "%u%%", telemetry->sensors.power.battery_percent);
    lv_label_set_text(child, buffer);
    child = lv_obj_get_child(bar, 4);
    lv_label_set_text(child, telemetry->sensors.power.charging ? "⚡" : " ");
}

void ui_apply_theme(lv_obj_t *obj) {
    if (!obj) {
        return;
    }
    lv_obj_set_style_bg_color(obj, lv_color_black(), 0);
    lv_obj_set_style_text_color(obj, lv_color_white(), 0);
}

