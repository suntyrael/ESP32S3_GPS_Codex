#include "ui_settings.h"

static const char *kOptions[SETTINGS_OPTION_COUNT] = {
    "IMU 校准",
    "磁力计校准",
    "GNSS 刷新率",
    "星座组合",
    "显示亮度",
    "关于"
};

lv_obj_t *ui_settings_create(lv_obj_t *parent) {
    lv_obj_t *screen = lv_obj_create(parent);
    ui_apply_theme(screen);
    lv_obj_set_size(screen, UI_SCREEN_WIDTH, UI_SCREEN_HEIGHT - UI_STATUS_BAR_HEIGHT);
    lv_obj_set_style_pad_all(screen, 5, 0);
    lv_obj_set_flex_flow(screen, LV_FLEX_FLOW_COLUMN);
    for (int i = 0; i < SETTINGS_OPTION_COUNT; ++i) {
        lv_obj_t *row = lv_label_create(screen);
        lv_label_set_text(row, kOptions[i]);
    }
    return screen;
}

void ui_settings_update(lv_obj_t *screen, settings_option_t selected, uint8_t gnss_rate_hz) {
    if (!screen) {
        return;
    }
    uint32_t count = lv_obj_get_child_cnt(screen);
    for (uint32_t i = 0; i < count; ++i) {
        lv_obj_t *row = lv_obj_get_child(screen, i);
        if (i == selected) {
            lv_obj_set_style_text_color(row, lv_color_hex(0x00ff00), 0);
        } else {
            lv_obj_set_style_text_color(row, lv_color_white(), 0);
        }
        if (i == SETTINGS_OPTION_GNSS_RATE) {
            char buffer[32];
            lv_snprintf(buffer, sizeof(buffer), "%s (%uHz)", kOptions[i], gnss_rate_hz);
            lv_label_set_text(row, buffer);
        } else {
            lv_label_set_text(row, kOptions[i]);
        }
    }
}

