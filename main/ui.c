/*
 * ui.c - LVGL 界面（阶段 4：多模式界面）
 * 模式（README §4）：码表 / GPS 记录 / P-Box / GNSS 信息 / 设置
 * 切换：编码器旋转（input_get_mode 轮询）；数据刷新 lv_timer 200ms
 */
#include "ui.h"
#include "ui_common.h"
#include "config.h"
#include "sensors.h"
#include "gnss.h"
#include "input.h"
#include "pbox.h"
#include "lcd_driver.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include <stdio.h>
#include <stdarg.h>

static const char *TAG = "ui";

/* ---- 每个模式的控件 ---- */
typedef struct {
    lv_obj_t *scr;
    lv_obj_t *status;   /* 状态栏 */
    lv_obj_t *big;      /* 主数值 */
    lv_obj_t *sub1;
    lv_obj_t *sub2;
    lv_obj_t *sub3;
} screen_t;

static screen_t s_scr[MODE_MAX];
static app_mode_t s_cur_mode = MODE_BIKE_COMPUTER;

static lv_obj_t *make_label(lv_obj_t *parent, lv_color_t color, const lv_font_t *font)
{
    lv_obj_t *lbl = lv_label_create(parent);
    lv_obj_set_style_text_color(lbl, color, 0);
    lv_obj_set_style_text_font(lbl, font, 0);
    return lbl;
}

static void set_big(lv_obj_t *lbl, const char *fmt, ...)
{
    char buf[64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    lv_label_set_text(lbl, buf);
}

/* ==================== 屏创建 ==================== */
static void make_status_bar(screen_t *sc, lv_obj_t *scr)
{
    sc->status = make_label(scr, UI_COL_SUB, &lv_font_montserrat_12);
    lv_obj_set_pos(sc->status, UI_PAD_S, UI_PAD_S);
}

static void create_bike_screen(void)
{
    screen_t *sc = &s_scr[MODE_BIKE_COMPUTER];
    sc->scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(sc->scr, UI_COL_BG, 0);
    lv_obj_remove_flag(sc->scr, LV_OBJ_FLAG_SCROLLABLE);
    make_status_bar(sc, sc->scr);
    sc->big = make_label(sc->scr, UI_COL_TEXT, &lv_font_montserrat_48);
    lv_obj_set_pos(sc->big, UI_PAD_M, 70);
    lv_label_set_text(sc->big, "0.0");
    sc->sub1 = make_label(sc->scr, UI_COL_TEXT, &lv_font_montserrat_16);
    lv_obj_set_pos(sc->sub1, UI_PAD_M, 180);
    sc->sub2 = make_label(sc->scr, UI_COL_SUB, &lv_font_montserrat_16);
    lv_obj_set_pos(sc->sub2, UI_PAD_M, 220);
    sc->sub3 = make_label(sc->scr, UI_COL_SUB, &lv_font_montserrat_16);
    lv_obj_set_pos(sc->sub3, UI_PAD_M, 260);
}

static void create_logger_screen(void)
{
    screen_t *sc = &s_scr[MODE_GPS_LOGGER];
    sc->scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(sc->scr, UI_COL_BG, 0);
    lv_obj_remove_flag(sc->scr, LV_OBJ_FLAG_SCROLLABLE);
    make_status_bar(sc, sc->scr);
    sc->big = make_label(sc->scr, UI_COL_TEXT, &lv_font_montserrat_48);
    lv_obj_set_pos(sc->big, UI_PAD_M, 60);
    sc->sub1 = make_label(sc->scr, UI_COL_TEXT, &lv_font_montserrat_16);
    lv_obj_set_pos(sc->sub1, UI_PAD_M, 180);
    sc->sub2 = make_label(sc->scr, UI_COL_SUB, &lv_font_montserrat_16);
    lv_obj_set_pos(sc->sub2, UI_PAD_M, 220);
    sc->sub3 = make_label(sc->scr, UI_COL_SUB, &lv_font_montserrat_16);
    lv_obj_set_pos(sc->sub3, UI_PAD_M, 260);
}

static void create_pbox_screen(void)
{
    screen_t *sc = &s_scr[MODE_PBOX];
    sc->scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(sc->scr, UI_COL_BG, 0);
    lv_obj_remove_flag(sc->scr, LV_OBJ_FLAG_SCROLLABLE);
    make_status_bar(sc, sc->scr);
    sc->sub1 = make_label(sc->scr, UI_COL_WARN, &lv_font_montserrat_16);   /* 状态提示 */
    lv_obj_set_pos(sc->sub1, UI_PAD_M, 25);
    sc->big = make_label(sc->scr, UI_COL_TEXT, &lv_font_montserrat_48);
    lv_obj_set_pos(sc->big, UI_PAD_M, 80);
    sc->sub2 = make_label(sc->scr, UI_COL_TEXT, &lv_font_montserrat_24);   /* 计时 */
    lv_obj_set_pos(sc->sub2, UI_PAD_M, 180);
    sc->sub3 = make_label(sc->scr, UI_COL_SUB, &lv_font_montserrat_16);
    lv_obj_set_pos(sc->sub3, UI_PAD_M, 240);
}

static void create_gnssinfo_screen(void)
{
    screen_t *sc = &s_scr[MODE_GNSS_INFO];
    sc->scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(sc->scr, UI_COL_BG, 0);
    lv_obj_remove_flag(sc->scr, LV_OBJ_FLAG_SCROLLABLE);
    make_status_bar(sc, sc->scr);
    sc->big = make_label(sc->scr, UI_COL_TEXT, &lv_font_montserrat_16);
    lv_obj_set_pos(sc->big, UI_PAD_M, 30);
    sc->sub1 = make_label(sc->scr, UI_COL_SUB, &lv_font_montserrat_16);
    lv_obj_set_pos(sc->sub1, UI_PAD_M, 90);
    sc->sub2 = make_label(sc->scr, UI_COL_SUB, &lv_font_montserrat_16);
    lv_obj_set_pos(sc->sub2, UI_PAD_M, 150);
    sc->sub3 = make_label(sc->scr, UI_COL_SUB, &lv_font_montserrat_16);
    lv_obj_set_pos(sc->sub3, UI_PAD_M, 210);
}

static void create_settings_screen(void)
{
    screen_t *sc = &s_scr[MODE_SETTINGS];
    sc->scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(sc->scr, UI_COL_BG, 0);
    lv_obj_remove_flag(sc->scr, LV_OBJ_FLAG_SCROLLABLE);
    make_status_bar(sc, sc->scr);
    sc->big = make_label(sc->scr, UI_COL_TEXT, &lv_font_montserrat_16);
    lv_obj_set_pos(sc->big, UI_PAD_M, 40);
    lv_label_set_text(sc->big, "SETTINGS (Stage 6)\n\n- IMU / MAG calib\n- GNSS rate\n- P-Box threshold\n- Brightness");
    sc->sub1 = make_label(sc->scr, UI_COL_SUB, &lv_font_montserrat_12);
    lv_obj_set_pos(sc->sub1, UI_PAD_M, 280);
    lv_label_set_text(sc->sub1, "FW " FW_VERSION_STR);
    sc->sub2 = NULL;
    sc->sub3 = NULL;
}

/* ==================== 数据刷新 ==================== */
static void update_status_bar(screen_t *sc, const gnss_data_t *g, const sensors_state_t *st)
{
    char sb[64];
    snprintf(sb, sizeof(sb), "GPS: %s%u  BAT: %u%%%s",
             g->valid ? "" : "no ", g->sats,
             st->battery.valid ? st->battery.percent : 0,
             st->battery.valid && st->battery.charging ? " CHG" : "");
    lv_label_set_text(sc->status, sb);
}

static void update_bike(const gnss_data_t *g, const sensors_state_t *st)
{
    screen_t *sc = &s_scr[MODE_BIKE_COMPUTER];
    set_big(sc->big, "%3.1f", g->valid ? g->speed_kmh : 0.0f);
    lv_label_set_text_fmt(sc->sub1, "ALT: %.0f m", st->baro.valid ? st->baro.altitude_m : 0.0f);
    lv_label_set_text_fmt(sc->sub2, "BARO: %.1f hPa", st->baro.valid ? st->baro.pressure_hpa : 0.0f);
    lv_label_set_text_fmt(sc->sub3, "IMU: %.1f C  MAG: %.1f C", 
                          st->imu.valid ? st->imu.temp_c : 0.0f,
                          st->mag.valid ? st->mag.temp_c : 0.0f);
    update_status_bar(sc, g, st);
}

static void update_logger(const gnss_data_t *g, const sensors_state_t *st)
{
    screen_t *sc = &s_scr[MODE_GPS_LOGGER];
    set_big(sc->big, "%3.1f", g->valid ? g->speed_kmh : 0.0f);
    lv_label_set_text_fmt(sc->sub1, "LAT: %.6f", g->lat);
    lv_label_set_text_fmt(sc->sub2, "LON: %.6f", g->lon);
    lv_label_set_text_fmt(sc->sub3, "ALT: %.0f m  SAT: %u", g->alt_m, g->sats);
    update_status_bar(sc, g, st);
}

static void update_pbox(const gnss_data_t *g, const sensors_state_t *st)
{
    (void)st;
    screen_t *sc = &s_scr[MODE_PBOX];
    pbox_status_t pb;
    pbox_get_status(&pb);
    const char *stxt = "READY";
    lv_color_t c = UI_COL_WARN;
    switch (pb.state) {
    case PBOX_READY:    stxt = "READY - press to arm"; break;
    case PBOX_ARMED:    stxt = pb.can_start ? "GO!" : "ARMED - wait GO"; c = UI_COL_GPS_OK; break;
    case PBOX_RUNNING:  stxt = "RUNNING"; c = UI_COL_REC; break;
    case PBOX_FINISHED: stxt = "TEST FINISHED!!!"; c = UI_COL_GPS_OK; break;
    }
    lv_obj_set_style_text_color(sc->sub1, c, 0);
    lv_label_set_text(sc->sub1, stxt);
    set_big(sc->big, "%3.1f", g->valid ? g->speed_kmh : 0.0f);
    lv_label_set_text_fmt(sc->sub2, "T: %.2f s", (double)pb.elapsed_s);
    lv_label_set_text_fmt(sc->sub3, "TARGET: 0-%.0f km/h  MAX: %.1f",
                          (double)pb.target_kmh, (double)pb.max_speed_kmh);
    update_status_bar(sc, g, st);
}

static void update_gnssinfo(const gnss_data_t *g, const sensors_state_t *st)
{
    screen_t *sc = &s_scr[MODE_GNSS_INFO];
    lv_label_set_text_fmt(sc->big, "LAT  %.6f\nLON  %.6f\nALT  %.0f m\nSPD  %.1f km/h",
                          g->lat, g->lon, g->alt_m, g->speed_kmh);
    lv_label_set_text_fmt(sc->sub1, "SAT  %u  FIX %u\nHDOP %.1f  VDOP %.1f  PDOP %.1f",
                          g->sats, g->fix_type, g->hdop, g->vdop, g->pdop);
    lv_label_set_text_fmt(sc->sub2, "IMU ACC %.2f %.2f %.2f g\nBARO %.1f hPa  %.0f m",
                          st->imu.valid ? st->imu.accel_mg[0] / 1000.0f : 0,
                          st->imu.valid ? st->imu.accel_mg[1] / 1000.0f : 0,
                          st->imu.valid ? st->imu.accel_mg[2] / 1000.0f : 0,
                          st->baro.valid ? st->baro.pressure_hpa : 0.0f,
                          st->baro.valid ? st->baro.altitude_m : 0.0f);
    lv_label_set_text_fmt(sc->sub3, "BAT %.2f V (%u%%)%s",
                          st->battery.valid ? st->battery.voltage_v : 0.0f,
                          st->battery.valid ? st->battery.percent : 0,
                          st->battery.valid && st->battery.charging ? " CHG" : "");
    update_status_bar(sc, g, st);
}

static void ui_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    gnss_data_t g;
    gnss_get_data(&g);
    sensors_state_t st;
    sensors_get_state(&st);

    /* 模式切换 */
    app_mode_t m = input_get_mode();
    if (m != s_cur_mode) {
        s_cur_mode = m;
        lv_screen_load(s_scr[m].scr);
    }
    /* 按当前模式更新 */
    switch (s_cur_mode) {
    case MODE_BIKE_COMPUTER: update_bike(&g, &st); break;
    case MODE_GPS_LOGGER:    update_logger(&g, &st); break;
    case MODE_PBOX:          update_pbox(&g, &st); break;
    case MODE_GNSS_INFO:     update_gnssinfo(&g, &st); break;
    default: break;   /* SETTINGS 静态 */
    }
}

/* ==================== 初始化 ==================== */
esp_err_t ui_init(void)
{
    const lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    esp_err_t ret = lvgl_port_init(&port_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "lvgl_port_init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_driver_get_io(),
        .panel_handle = lcd_driver_get_panel(),
        .buffer_size = UI_H_RES * UI_V_RES / 10,
        .double_buffer = true,
        .hres = UI_H_RES,
        .vres = UI_V_RES,
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,
            .mirror_y = true,
        },
        .flags = {
            .buff_dma = true,
            .swap_bytes = true,
        },
    };
    if (lvgl_port_add_disp(&disp_cfg) == NULL) {
        ESP_LOGE(TAG, "lvgl_port_add_disp failed");
        return ESP_ERR_INVALID_STATE;
    }

    lvgl_port_lock(0);
    create_bike_screen();
    create_logger_screen();
    create_pbox_screen();
    create_gnssinfo_screen();
    create_settings_screen();
    lv_screen_load(s_scr[MODE_BIKE_COMPUTER].scr);
    lvgl_port_unlock();

    lv_timer_create(ui_timer_cb, UI_REFRESH_PERIOD_MS, NULL);
    ESP_LOGI(TAG, "LVGL %d.%d multi-mode UI ready", LVGL_VERSION_MAJOR, LVGL_VERSION_MINOR);
    return ESP_OK;
}

bool ui_lock(void)   { return lvgl_port_lock(100); }
void ui_unlock(void) { lvgl_port_unlock(); }
