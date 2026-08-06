/*
 * ui.c - LVGL 界面实现（阶段 2）
 * 布局：
 *   [状态栏 20px]  GPS 卫星数 | SD | BAT %
 *   [主区]         IMU / MAG / BARO / TEMP / BAT 数据
 * 数据源：sensors_get_state() 线程安全快照（C-08）
 */
#include "ui.h"
#include "ui_common.h"
#include "config.h"
#include "sensors.h"
#include "lcd_driver.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"

static const char *TAG = "ui";

static lv_obj_t *s_lbl_status = NULL;   /* 状态栏 */
static lv_obj_t *s_lbl_imu = NULL;
static lv_obj_t *s_lbl_mag = NULL;
static lv_obj_t *s_lbl_baro = NULL;
static lv_obj_t *s_lbl_bat = NULL;
static lv_obj_t *s_lbl_gnss = NULL;

static lv_obj_t *make_label(lv_obj_t *parent, lv_color_t color, uint8_t px)
{
    lv_obj_t *lbl = lv_label_create(parent);
    lv_obj_set_style_text_color(lbl, color, 0);
    lv_obj_set_style_text_font(lbl, px <= 12 ? &lv_font_montserrat_12 : &lv_font_montserrat_16, 0);
    return lbl;
}

static void ui_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    sensors_state_t st;
    sensors_get_state(&st);

    /* 状态栏：GNSS 卫星数（阶段 3 前固定 N/A）+ 电池 */
    char sb[64];
    snprintf(sb, sizeof(sb), "GPS: N/A   BAT: %u%%%s",
             st.battery.valid ? st.battery.percent : 0,
             st.battery.valid && st.battery.charging ? " CHG" : "");
    lv_label_set_text(s_lbl_status, sb);

    if (st.imu.valid) {
        lv_label_set_text_fmt(s_lbl_imu,
                              "IMU  ACC: %.2f %.2f %.2f g\n"
                              "     LIN: %.2f %.2f %.2f g\n"
                              "     GYRO: %.1f %.1f %.1f dps",
                              st.imu.accel_mg[0] / 1000.0f, st.imu.accel_mg[1] / 1000.0f, st.imu.accel_mg[2] / 1000.0f,
                              st.imu.lin_mg[0] / 1000.0f, st.imu.lin_mg[1] / 1000.0f, st.imu.lin_mg[2] / 1000.0f,
                              st.imu.gyro_mdps[0] / 1000.0f, st.imu.gyro_mdps[1] / 1000.0f, st.imu.gyro_mdps[2] / 1000.0f);
    } else {
        lv_label_set_text(s_lbl_imu, "IMU: --");
    }
    if (st.mag.valid) {
        lv_label_set_text_fmt(s_lbl_mag, "MAG: %.1f %.1f %.1f mG",
                              st.mag.mag_mgauss[0], st.mag.mag_mgauss[1], st.mag.mag_mgauss[2]);
    } else {
        lv_label_set_text(s_lbl_mag, "MAG: --");
    }
    if (st.baro.valid) {
        lv_label_set_text_fmt(s_lbl_baro, "BARO: %.1f hPa  %.1f m\nTEMP: IMU %.1f  BARO %.1f  MAG %.1f C",
                              st.baro.pressure_hpa, st.baro.altitude_m,
                              st.imu.valid ? st.imu.temp_c : 0.0f,
                              st.baro.temp_c,
                              st.mag.valid ? st.mag.temp_c : 0.0f);
    } else {
        lv_label_set_text(s_lbl_baro, "BARO: --");
    }
    if (st.battery.valid) {
        lv_label_set_text_fmt(s_lbl_bat, "BAT: %.2f V (%u%%)",
                              st.battery.voltage_v, st.battery.percent);
    } else {
        lv_label_set_text(s_lbl_bat, "BAT: --");
    }
    (void)s_lbl_gnss;
}

esp_err_t ui_init(void)
{
    /* LVGL port：初始化 LVGL + 自带任务/timer */
    const lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    esp_err_t ret = lvgl_port_init(&port_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "lvgl_port_init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_driver_get_io(),
        .panel_handle = lcd_driver_get_panel(),
        .buffer_size = UI_H_RES * UI_V_RES / 10,    /* 1/10 屏，双缓冲 */
        .double_buffer = true,
        .hres = UI_H_RES,
        .vres = UI_V_RES,
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,                       /* 旋转 180°：X/Y 均镜像 */
            .mirror_y = true,
        },
        .flags = {
            .buff_dma = true,
            .swap_bytes = true,                     /* RGB565 字节序交换 */
        },
    };
    if (lvgl_port_add_disp(&disp_cfg) == NULL) {
        ESP_LOGE(TAG, "lvgl_port_add_disp failed");
        return ESP_ERR_INVALID_STATE;
    }

    lvgl_port_lock(0);
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, UI_COL_BG, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    /* 状态栏 */
    s_lbl_status = make_label(scr, UI_COL_SUB, UI_FONT_SMALL);
    lv_obj_set_pos(s_lbl_status, UI_PAD_M, UI_PAD_S);

    /* 主区数据 */
    s_lbl_imu = make_label(scr, UI_COL_TEXT, UI_FONT_MEDIUM);
    lv_obj_set_pos(s_lbl_imu, UI_PAD_M, UI_STATUS_H + UI_PAD_M);
    s_lbl_mag = make_label(scr, UI_COL_SUB, UI_FONT_MEDIUM);
    lv_obj_set_pos(s_lbl_mag, UI_PAD_M, UI_STATUS_H + UI_PAD_M + 64);
    s_lbl_baro = make_label(scr, UI_COL_TEXT, UI_FONT_MEDIUM);
    lv_obj_set_pos(s_lbl_baro, UI_PAD_M, UI_STATUS_H + UI_PAD_M + 96);
    s_lbl_bat = make_label(scr, UI_COL_SUB, UI_FONT_MEDIUM);
    lv_obj_set_pos(s_lbl_bat, UI_PAD_M, UI_STATUS_H + UI_PAD_M + 160);
    lvgl_port_unlock();

    /* 周期刷新 */
    lv_timer_create(ui_timer_cb, UI_REFRESH_PERIOD_MS, NULL);

    ESP_LOGI(TAG, "LVGL %d.%d UI ready", LVGL_VERSION_MAJOR, LVGL_VERSION_MINOR);
    return ESP_OK;
}

bool ui_lock(void)   { return lvgl_port_lock(100); }
void ui_unlock(void) { lvgl_port_unlock(); }
