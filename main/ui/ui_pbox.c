#include "ui_pbox.h"

#include <stdio.h>

lv_obj_t *ui_pbox_create(lv_obj_t *parent) {
    lv_obj_t *screen = lv_obj_create(parent);
    ui_apply_theme(screen);
    lv_obj_set_size(screen, UI_SCREEN_WIDTH, UI_SCREEN_HEIGHT - UI_STATUS_BAR_HEIGHT);
    lv_obj_set_style_pad_all(screen, 5, 0);
    lv_obj_set_flex_flow(screen, LV_FLEX_FLOW_COLUMN);
    lv_obj_t *speed = lv_label_create(screen);
    lv_obj_set_style_text_font(speed, &lv_font_montserrat_48, 0);
    lv_label_set_text(speed, "000.0 km/h");
    lv_obj_t *timer = lv_label_create(screen);
    lv_obj_set_style_text_font(timer, &lv_font_montserrat_32, 0);
    lv_label_set_text(timer, "00.000 s");
    lv_obj_t *target = lv_label_create(screen);
    lv_label_set_text(target, "目标 0-100 km/h");
    lv_obj_t *status = lv_label_create(screen);
    lv_label_set_text(status, "READY");
    return screen;
}

void ui_pbox_update(lv_obj_t *screen, const ui_telemetry_t *telemetry,
                    pbox_status_t status, float current_target_kmh, float elapsed_s) {
    if (!screen || !telemetry) {
        return;
    }
    char buffer[48];
    lv_obj_t *speed = lv_obj_get_child(screen, 0);
    snprintf(buffer, sizeof(buffer), "%05.1f km/h", telemetry->sensors.gnss.speed_kmh);
    lv_label_set_text(speed, buffer);
    lv_obj_t *timer = lv_obj_get_child(screen, 1);
    snprintf(buffer, sizeof(buffer), "%06.3f s", elapsed_s);
    lv_label_set_text(timer, buffer);
    lv_obj_t *target = lv_obj_get_child(screen, 2);
    snprintf(buffer, sizeof(buffer), "目标 0-%.0f km/h", current_target_kmh);
    lv_label_set_text(target, buffer);
    lv_obj_t *status_label = lv_obj_get_child(screen, 3);
    static const char *status_text[] = {"READY", "ARMED", "RUN", "FINISHED"};
    lv_label_set_text(status_label, status_text[status]);
}

