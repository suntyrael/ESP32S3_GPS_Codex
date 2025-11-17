#include "ui_gnss_info.h"

#include <stdio.h>

static void update_sat_list(lv_obj_t *list, const gnss_state_t *gnss) {
    lv_obj_clean(list);
    char buffer[48];
    for (int i = 0; i < gnss->sats_in_view && i < GNSS_MAX_SATELLITES; ++i) {
        const gnss_satellite_t *sat = &gnss->satellites[i];
        snprintf(buffer, sizeof(buffer), "%03u %d %.0f %.0f", sat->id, sat->constellation, sat->cn0_dbhz, sat->elevation_deg);
        lv_obj_t *label = lv_label_create(list);
        lv_label_set_text(label, buffer);
    }
}

lv_obj_t *ui_gnss_info_create(lv_obj_t *parent) {
    lv_obj_t *screen = lv_obj_create(parent);
    ui_apply_theme(screen);
    lv_obj_set_size(screen, UI_SCREEN_WIDTH, UI_SCREEN_HEIGHT - UI_STATUS_BAR_HEIGHT);
    lv_obj_set_style_pad_all(screen, 5, 0);
    lv_obj_set_flex_flow(screen, LV_FLEX_FLOW_COLUMN);
    lv_obj_t *fix = lv_label_create(screen);
    lv_obj_t *dop = lv_label_create(screen);
    lv_obj_t *list = lv_obj_create(screen);
    lv_obj_set_size(list, UI_SCREEN_WIDTH - 10, 200);
    lv_obj_set_style_bg_color(list, lv_color_hex(0x101010), 0);
    lv_obj_set_style_pad_all(list, 2, 0);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);
    (void)fix;
    (void)dop;
    return screen;
}

void ui_gnss_info_update(lv_obj_t *screen, const ui_telemetry_t *telemetry) {
    if (!screen || !telemetry) {
        return;
    }
    const gnss_state_t *gnss = &telemetry->sensors.gnss;
    char buffer[64];
    lv_obj_t *fix = lv_obj_get_child(screen, 0);
    snprintf(buffer, sizeof(buffer), "Lat %.5f Lon %.5f Alt %.1f Speed %.1f", gnss->latitude_deg, gnss->longitude_deg,
             gnss->altitude_m, gnss->speed_kmh);
    lv_label_set_text(fix, buffer);
    lv_obj_t *dop = lv_obj_get_child(screen, 1);
    snprintf(buffer, sizeof(buffer), "HDOP %.1f VDOP %.1f PDOP %.1f", gnss->hdop, gnss->vdop, gnss->pdop);
    lv_label_set_text(dop, buffer);
    lv_obj_t *list = lv_obj_get_child(screen, 2);
    update_sat_list(list, gnss);
}

