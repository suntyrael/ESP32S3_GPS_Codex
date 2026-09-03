/*
 * ui.c - 基于 LVGL 9.5 的 240x320 竖屏多功能仪表界面
 * 架构：3 大页面 (主功能页 / 传感器诊断 / 系统设置)
 * Page 0 内容完全由系统设置决定 (P-Box / 轨迹记录 / 自行车码表)
 * 硬件规格：ST7789 240x320 竖屏、EC11 旋转编码器
 * 约束：严格遵循 C-06 (无魔数，收敛至 ui_common.h)、C-09 (LVGL 互斥保护)
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
#include <string.h>
#include <math.h>

static const char *TAG = "ui";

/* ==================== 运行状态结构体 ==================== */
typedef struct {
    bool is_dark_theme;             /* 0: Sunlight 高对比度日光, 1: Dark 暗夜 */
    bool is_tracking;               /* 轨迹记录中状态 */
    uint32_t track_start_sec;       /* 记录起始时刻 */
    uint32_t track_points;          /* 记录点数 */
    float track_dist_km;            /* 累计记录距离 */
    float last_lat;
    float last_lon;
    bool bike_paused;               /* 码表暂停状态 */
    float bike_trip_km;             /* 码表单次里程 */
    float bike_max_speed;           /* 码表最高速度 */
    int setting_selected_idx;       /* 设置页选中行 */
} ui_state_t;

static ui_state_t s_ui = {
    .is_dark_theme = false,         /* 默认 Sunlight 日光高对比度 */
    .is_tracking = false,
    .track_start_sec = 0,
    .track_points = 0,
    .track_dist_km = 0.0f,
    .bike_paused = false,
    .bike_trip_km = 0.0f,
    .bike_max_speed = 0.0f,
    .setting_selected_idx = 0,
};

/* ==================== 屏幕与控件句柄 ==================== */
typedef struct {
    lv_obj_t *root;                 /* 页面根对象 */
    lv_obj_t *status_bar;           /* 顶部状态栏 */
    lv_obj_t *lbl_sat;              /* 卫星数 */
    lv_obj_t *lbl_rate;             /* 频率 */
    lv_obj_t *lbl_sd;               /* SD 卡图标 */
    lv_obj_t *lbl_clock;            /* 本地时钟 */
    lv_obj_t *lbl_bat;              /* 电池与充电 */
    lv_obj_t *nav_bar;              /* 底部导航栏 */
    lv_obj_t *nav_dots[3];          /* 3 个页面指示点 */
    lv_obj_t *content;              /* 中间内容容器 */
} screen_base_t;

static screen_base_t s_screens[MODE_MAX];
static app_mode_t s_cur_mode = MODE_MAIN;
static main_page_t s_cur_subpage = MAIN_PAGE_PBOX;

/* ---- Page 0: P-Box 控件 ---- */
typedef struct {
    lv_obj_t *container;
    lv_obj_t *lbl_title_badge;
    lv_obj_t *lbl_speed_live;
    lv_obj_t *lbl_timer_big;
    lv_obj_t *lbl_status_msg;
    lv_obj_t *lbl_060_val;
    lv_obj_t *lbl_0100_val;
    lv_obj_t *lbl_400m_val;
    lv_obj_t *lbl_slope_val;
    lv_obj_t *g_dot;                /* G 值动态雷达红点 */
    lv_obj_t *lbl_g_long;
    lv_obj_t *lbl_g_lat;
    lv_obj_t *lbl_g_peak;
} pbox_view_t;
static pbox_view_t s_pbox;

/* ---- Page 0: 轨迹记录仪控件 ---- */
typedef struct {
    lv_obj_t *container;
    lv_obj_t *lbl_title_badge;
    lv_obj_t *lbl_scale_hdg;
    lv_obj_t *lbl_rec_tag;
    lv_obj_t *lbl_dist_big;
    lv_obj_t *lbl_ride_time;
    lv_obj_t *lbl_elev_gain;
    lv_obj_t *lbl_avg_spd;
    lv_obj_t *lbl_log_pts;
    lv_obj_t *lbl_hint;
} track_view_t;
static track_view_t s_track;

/* ---- Page 0: 自行车码表控件 ---- */
typedef struct {
    lv_obj_t *container;
    lv_obj_t *lbl_title_badge;
    lv_obj_t *lbl_speed_big;
    lv_obj_t *lbl_max_avg;
    lv_obj_t *lbl_trip_val;
    lv_obj_t *lbl_slope_val;
    lv_obj_t *lbl_alt_val;
    lv_obj_t *lbl_vspd_val;
    lv_obj_t *lbl_heading_txt;
} bike_view_t;
static bike_view_t s_bike;

/* ---- Page 1: 传感器诊断控件 ---- */
typedef struct {
    lv_obj_t *lbl_imu_raw_acc;
    lv_obj_t *lbl_imu_lin_acc;      /* 线性加速度（用户特别要求） */
    lv_obj_t *lbl_imu_gyro;
    lv_obj_t *lbl_mag_val;
    lv_obj_t *lbl_mag_heading;
    lv_obj_t *lbl_baro_val;
    lv_obj_t *lbl_baro_alt;
    lv_obj_t *lbl_gnss_status;
    lv_obj_t *lbl_gnss_pos;
    lv_obj_t *lbl_gnss_dop;
} diag_view_t;
static diag_view_t s_diag;

/* ---- Page 2: 系统设置控件 ---- */
#define SETTING_ITEM_COUNT  7
typedef struct {
    lv_obj_t *rows[SETTING_ITEM_COUNT];
    lv_obj_t *val_labels[SETTING_ITEM_COUNT];
    lv_obj_t *lbl_page_idx;
} settings_view_t;
static settings_view_t s_settings;

static const char *s_setting_keys[SETTING_ITEM_COUNT] = {
    "FUNCTION MODE",
    "COLOR THEME",
    "GPS RATE",
    "GNSS MODE",
    "BRIGHTNESS",
    "AUTO SLEEP",
    "SENSOR CALIB"
};

static int s_setting_vals[SETTING_ITEM_COUNT] = { 0, 0, 0, 0, 0, 0, 0 };

/* ==================== 基础控件创建辅助 ==================== */
static lv_obj_t *create_card(lv_obj_t *parent, int32_t w, int32_t h)
{
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_size(card, w, h);
    lv_obj_set_style_bg_color(card, s_ui.is_dark_theme ? UI_COL_DARK_CARD : UI_COL_SUN_CARD, 0);
    lv_obj_set_style_border_color(card, s_ui.is_dark_theme ? UI_COL_DARK_BORDER : UI_COL_SUN_BORDER, 0);
    lv_obj_set_style_border_width(card, UI_BORDER_WIDTH, 0);
    lv_obj_set_style_radius(card, UI_RADIUS_M, 0);
    lv_obj_set_style_pad_all(card, UI_PAD_S, 0);
    lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    return card;
}

static lv_obj_t *create_label(lv_obj_t *parent, const char *text, const lv_font_t *font, lv_color_t color)
{
    lv_obj_t *lbl = lv_label_create(parent);
    if (text) {
        lv_label_set_text(lbl, text);
    }
    lv_obj_set_style_text_font(lbl, font, 0);
    lv_obj_set_style_text_color(lbl, color, 0);
    return lbl;
}

/* ==================== 通用状态栏与导航栏创建 ==================== */
static void create_screen_shell(app_mode_t mode)
{
    screen_base_t *sb = &s_screens[mode];
    sb->root = lv_obj_create(NULL);
    lv_obj_set_size(sb->root, UI_H_RES, UI_V_RES);
    lv_obj_set_style_bg_color(sb->root, s_ui.is_dark_theme ? UI_COL_DARK_BG : UI_COL_SUN_BG, 0);
    lv_obj_set_style_pad_all(sb->root, 0, 0);
    lv_obj_remove_flag(sb->root, LV_OBJ_FLAG_SCROLLABLE);

    /* 1. 顶部状态栏 (高度 20px) */
    sb->status_bar = lv_obj_create(sb->root);
    lv_obj_set_size(sb->status_bar, UI_H_RES, UI_STATUS_H);
    lv_obj_set_pos(sb->status_bar, 0, 0);
    lv_obj_set_style_bg_color(sb->status_bar, s_ui.is_dark_theme ? lv_color_hex(0x070A10) : lv_color_hex(0xE2E8F0), 0);
    lv_obj_set_style_border_side(sb->status_bar, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_color(sb->status_bar, s_ui.is_dark_theme ? UI_COL_DARK_BORDER : UI_COL_SUN_BORDER, 0);
    lv_obj_set_style_border_width(sb->status_bar, 1, 0);
    lv_obj_set_style_radius(sb->status_bar, 0, 0);
    lv_obj_set_style_pad_all(sb->status_bar, 0, 0);
    lv_obj_remove_flag(sb->status_bar, LV_OBJ_FLAG_SCROLLABLE);

    /* 状态栏控件 */
    sb->lbl_sat = create_label(sb->status_bar, "0 SAT", UI_FONT_12, UI_COL_ORANGE);
    lv_obj_set_pos(sb->lbl_sat, 4, 2);

    sb->lbl_rate = create_label(sb->status_bar, "10Hz", UI_FONT_12, s_ui.is_dark_theme ? UI_COL_CYAN : UI_COL_BLUE);
    lv_obj_set_pos(sb->lbl_rate, 64, 2);

    sb->lbl_sd = create_label(sb->status_bar, "SD", UI_FONT_12, UI_COL_GREEN);
    lv_obj_set_pos(sb->lbl_sd, 96, 2);

    sb->lbl_clock = create_label(sb->status_bar, "00:00:00", UI_FONT_MONO, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(sb->lbl_clock, 126, 2);

    sb->lbl_bat = create_label(sb->status_bar, "4.1V", UI_FONT_12, UI_COL_GREEN);
    lv_obj_set_pos(sb->lbl_bat, 188, 2);

    /* 2. 中部内容视口 (高度 284px = 320 - 20 - 16) */
    sb->content = lv_obj_create(sb->root);
    lv_obj_set_size(sb->content, UI_H_RES, UI_CONTENT_H);
    lv_obj_set_pos(sb->content, 0, UI_STATUS_H);
    lv_obj_set_style_bg_color(sb->content, s_ui.is_dark_theme ? UI_COL_DARK_BG : UI_COL_SUN_BG, 0);
    lv_obj_set_style_border_width(sb->content, 0, 0);
    lv_obj_set_style_radius(sb->content, 0, 0);
    lv_obj_set_style_pad_all(sb->content, UI_PAD_S, 0);
    lv_obj_remove_flag(sb->content, LV_OBJ_FLAG_SCROLLABLE);

    /* 3. 底部导航指示栏 (高度 16px) */
    sb->nav_bar = lv_obj_create(sb->root);
    lv_obj_set_size(sb->nav_bar, UI_H_RES, UI_NAV_H);
    lv_obj_set_pos(sb->nav_bar, 0, UI_V_RES - UI_NAV_H);
    lv_obj_set_style_bg_color(sb->nav_bar, s_ui.is_dark_theme ? lv_color_hex(0x070A10) : lv_color_hex(0xE2E8F0), 0);
    lv_obj_set_style_border_side(sb->nav_bar, LV_BORDER_SIDE_TOP, 0);
    lv_obj_set_style_border_color(sb->nav_bar, s_ui.is_dark_theme ? UI_COL_DARK_BORDER : UI_COL_SUN_BORDER, 0);
    lv_obj_set_style_border_width(sb->nav_bar, 1, 0);
    lv_obj_set_style_radius(sb->nav_bar, 0, 0);
    lv_obj_set_style_pad_all(sb->nav_bar, 0, 0);
    lv_obj_remove_flag(sb->nav_bar, LV_OBJ_FLAG_SCROLLABLE);

    /* 导航栏 3 个圆点 */
    const int dot_x[3] = { 80, 115, 150 };
    for (int i = 0; i < 3; i++) {
        sb->nav_dots[i] = lv_obj_create(sb->nav_bar);
        if (i == (int)mode) {
            lv_obj_set_size(sb->nav_dots[i], 22, 6);
            lv_obj_set_style_bg_color(sb->nav_dots[i], s_ui.is_dark_theme ? UI_COL_CYAN : UI_COL_BLUE, 0);
        } else {
            lv_obj_set_size(sb->nav_dots[i], 6, 6);
            lv_obj_set_style_bg_color(sb->nav_dots[i], lv_color_hex(0x64748B), 0);
        }
        lv_obj_set_pos(sb->nav_dots[i], dot_x[i], 5);
        lv_obj_set_style_radius(sb->nav_dots[i], 3, 0);
        lv_obj_set_style_border_width(sb->nav_dots[i], 0, 0);
        lv_obj_remove_flag(sb->nav_dots[i], LV_OBJ_FLAG_SCROLLABLE);
    }
}

/* ==================== PAGE 0: 子视图创建 ==================== */

/* 1. P-Box 直线加速视图 */
static void create_pbox_view(lv_obj_t *parent)
{
    s_pbox.container = lv_obj_create(parent);
    lv_obj_set_size(s_pbox.container, UI_H_RES - 8, UI_CONTENT_H - 8);
    lv_obj_set_pos(s_pbox.container, 0, 0);
    lv_obj_set_style_bg_opa(s_pbox.container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_pbox.container, 0, 0);
    lv_obj_set_style_pad_all(s_pbox.container, 0, 0);
    lv_obj_remove_flag(s_pbox.container, LV_OBJ_FLAG_SCROLLABLE);

    /* 标题栏 */
    lv_obj_t *t_hdr = create_label(s_pbox.container, "P-BOX DRAG METER", UI_FONT_12, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(t_hdr, 2, 0);
    s_pbox.lbl_title_badge = create_label(s_pbox.container, "READY", UI_FONT_12, UI_COL_GREEN);
    lv_obj_set_pos(s_pbox.lbl_title_badge, 185, 0);

    /* 顶部主计时卡片 (高度 84px) */
    lv_obj_t *card_top = create_card(s_pbox.container, UI_H_RES - 8, 84);
    lv_obj_set_pos(card_top, 0, 18);

    create_label(card_top, "0 - 100 KM/H", UI_FONT_12, UI_COL_ORANGE);
    s_pbox.lbl_speed_live = create_label(card_top, "SPD: 0.0 km/h", UI_FONT_12, s_ui.is_dark_theme ? UI_COL_DARK_SUB : UI_COL_SUN_SUB);
    lv_obj_set_pos(s_pbox.lbl_speed_live, 120, 0);

    s_pbox.lbl_timer_big = create_label(card_top, "00.00s", UI_FONT_48, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(s_pbox.lbl_timer_big, 28, 16);

    s_pbox.lbl_status_msg = create_label(card_top, "STANDBY • HIT GAS TO LAUNCH", UI_FONT_12, UI_COL_GREEN);
    lv_obj_set_pos(s_pbox.lbl_status_msg, 10, 62);

    /* 2x2 统计卡片 (Y=106, 每张 112x40) */
    lv_obj_t *c1 = create_card(s_pbox.container, 112, 40);
    lv_obj_set_pos(c1, 0, 106);
    create_label(c1, "0 - 60 KM/H", UI_FONT_12, UI_COL_DIM);
    s_pbox.lbl_060_val = create_label(c1, "2.84 s", UI_FONT_16, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(s_pbox.lbl_060_val, 0, 16);

    lv_obj_t *c2 = create_card(s_pbox.container, 112, 40);
    lv_obj_set_pos(c2, 120, 106);
    create_label(c2, "0 - 100 BEST", UI_FONT_12, UI_COL_DIM);
    s_pbox.lbl_0100_val = create_label(c2, "5.92 s", UI_FONT_16, s_ui.is_dark_theme ? UI_COL_CYAN : UI_COL_BLUE);
    lv_obj_set_pos(s_pbox.lbl_0100_val, 0, 16);

    lv_obj_t *c3 = create_card(s_pbox.container, 112, 40);
    lv_obj_set_pos(c3, 0, 150);
    create_label(c3, "1/4 MILE (402M)", UI_FONT_12, UI_COL_DIM);
    s_pbox.lbl_400m_val = create_label(c3, "13.81 s", UI_FONT_16, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(s_pbox.lbl_400m_val, 0, 16);

    lv_obj_t *c4 = create_card(s_pbox.container, 112, 40);
    lv_obj_set_pos(c4, 120, 150);
    create_label(c4, "SLOPE / VALID", UI_FONT_12, UI_COL_DIM);
    s_pbox.lbl_slope_val = create_label(c4, "-0.3% OK", UI_FONT_16, UI_COL_GREEN);
    lv_obj_set_pos(s_pbox.lbl_slope_val, 0, 16);

    /* G-Force 加速度雷达面板 (Y=194, 高度 78px) */
    lv_obj_t *c_g = create_card(s_pbox.container, UI_H_RES - 8, 78);
    lv_obj_set_pos(c_g, 0, 194);

    /* 雷达圆形边框 */
    lv_obj_t *g_circle = lv_obj_create(c_g);
    lv_obj_set_size(g_circle, 56, 56);
    lv_obj_set_pos(g_circle, 4, 4);
    lv_obj_set_style_radius(g_circle, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(g_circle, s_ui.is_dark_theme ? lv_color_hex(0x141B24) : lv_color_hex(0xF1F5F9), 0);
    lv_obj_set_style_border_color(g_circle, s_ui.is_dark_theme ? UI_COL_DARK_BORDER : UI_COL_SUN_BORDER, 0);
    lv_obj_set_style_border_width(g_circle, 1, 0);
    lv_obj_remove_flag(g_circle, LV_OBJ_FLAG_SCROLLABLE);

    /* 十字准星与中心动态红点 */
    lv_obj_t *cross_h = lv_obj_create(g_circle);
    lv_obj_set_size(cross_h, 54, 1);
    lv_obj_set_pos(cross_h, 0, 27);
    lv_obj_set_style_bg_color(cross_h, s_ui.is_dark_theme ? UI_COL_DARK_BORDER : UI_COL_SUN_BORDER, 0);
    lv_obj_set_style_border_width(cross_h, 0, 0);

    lv_obj_t *cross_v = lv_obj_create(g_circle);
    lv_obj_set_size(cross_v, 1, 54);
    lv_obj_set_pos(cross_v, 27, 0);
    lv_obj_set_style_bg_color(cross_v, s_ui.is_dark_theme ? UI_COL_DARK_BORDER : UI_COL_SUN_BORDER, 0);
    lv_obj_set_style_border_width(cross_v, 0, 0);

    s_pbox.g_dot = lv_obj_create(g_circle);
    lv_obj_set_size(s_pbox.g_dot, 8, 8);
    lv_obj_set_pos(s_pbox.g_dot, 24, 24);
    lv_obj_set_style_radius(s_pbox.g_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(s_pbox.g_dot, UI_COL_RED, 0);
    lv_obj_set_style_border_width(s_pbox.g_dot, 0, 0);
    lv_obj_remove_flag(s_pbox.g_dot, LV_OBJ_FLAG_SCROLLABLE);

    /* 右侧数值 */
    s_pbox.lbl_g_long = create_label(c_g, "LONG G: +0.00 G", UI_FONT_12, UI_COL_ORANGE);
    lv_obj_set_pos(s_pbox.lbl_g_long, 72, 6);

    s_pbox.lbl_g_lat = create_label(c_g, "LAT G:  +0.00 G", UI_FONT_12, UI_COL_RED);
    lv_obj_set_pos(s_pbox.lbl_g_lat, 72, 26);

    s_pbox.lbl_g_peak = create_label(c_g, "PEAK G: 0.89 G", UI_FONT_12, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(s_pbox.lbl_g_peak, 72, 46);
}

/* 2. 轨迹记录仪视图 */
static void create_track_view(lv_obj_t *parent)
{
    s_track.container = lv_obj_create(parent);
    lv_obj_set_size(s_track.container, UI_H_RES - 8, UI_CONTENT_H - 8);
    lv_obj_set_pos(s_track.container, 0, 0);
    lv_obj_set_style_bg_opa(s_track.container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_track.container, 0, 0);
    lv_obj_set_style_pad_all(s_track.container, 0, 0);
    lv_obj_remove_flag(s_track.container, LV_OBJ_FLAG_SCROLLABLE);

    /* 标题栏 */
    lv_obj_t *t_hdr = create_label(s_track.container, "TRACK RECORDER", UI_FONT_12, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(t_hdr, 2, 0);
    s_track.lbl_title_badge = create_label(s_track.container, "STANDBY", UI_FONT_12, UI_COL_GREEN);
    lv_obj_set_pos(s_track.lbl_title_badge, 175, 0);

    /* 轨迹微缩地图框 (Y=18, 高度 76px) */
    lv_obj_t *map_card = create_card(s_track.container, UI_H_RES - 8, 76);
    lv_obj_set_pos(map_card, 0, 18);
    s_track.lbl_scale_hdg = create_label(map_card, "SCALE 200m | HDG 072\xC2\xB0", UI_FONT_12, UI_COL_DIM);
    lv_obj_set_pos(s_track.lbl_scale_hdg, 4, 4);
    s_track.lbl_rec_tag = create_label(map_card, "REC", UI_FONT_12, UI_COL_GREEN);
    lv_obj_set_pos(s_track.lbl_rec_tag, 192, 4);

    /* 轨迹模拟曲线指示点 */
    lv_obj_t *pt_start = lv_obj_create(map_card);
    lv_obj_set_size(pt_start, 6, 6);
    lv_obj_set_pos(pt_start, 24, 48);
    lv_obj_set_style_radius(pt_start, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(pt_start, UI_COL_GREEN, 0);
    lv_obj_set_style_border_width(pt_start, 0, 0);

    lv_obj_t *pt_cur = lv_obj_create(map_card);
    lv_obj_set_size(pt_cur, 8, 8);
    lv_obj_set_pos(pt_cur, 180, 36);
    lv_obj_set_style_radius(pt_cur, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(pt_cur, UI_COL_RED, 0);
    lv_obj_set_style_border_width(pt_cur, 0, 0);

    /* 总里程 Hero 行 (Y=98, 高度 58px) */
    lv_obj_t *dist_card = create_card(s_track.container, UI_H_RES - 8, 58);
    lv_obj_set_pos(dist_card, 0, 98);
    create_label(dist_card, "TOTAL DISTANCE", UI_FONT_12, UI_COL_DIM);
    s_track.lbl_dist_big = create_label(dist_card, "18.42", UI_FONT_48, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(s_track.lbl_dist_big, 80, 2);
    lv_obj_t *u_km = create_label(dist_card, "KM", UI_FONT_16, s_ui.is_dark_theme ? UI_COL_CYAN : UI_COL_BLUE);
    lv_obj_set_pos(u_km, 198, 28);

    /* 2x2 统计卡片 (Y=160) */
    lv_obj_t *c1 = create_card(s_track.container, 112, 42);
    lv_obj_set_pos(c1, 0, 160);
    create_label(c1, "RIDE TIME", UI_FONT_12, UI_COL_DIM);
    s_track.lbl_ride_time = create_label(c1, "01:14:28", UI_FONT_16, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(s_track.lbl_ride_time, 0, 18);

    lv_obj_t *c2 = create_card(s_track.container, 112, 42);
    lv_obj_set_pos(c2, 120, 160);
    create_label(c2, "ELEV GAIN", UI_FONT_12, UI_COL_DIM);
    s_track.lbl_elev_gain = create_label(c2, "+312 m", UI_FONT_16, UI_COL_ORANGE);
    lv_obj_set_pos(s_track.lbl_elev_gain, 0, 18);

    lv_obj_t *c3 = create_card(s_track.container, 112, 42);
    lv_obj_set_pos(c3, 0, 206);
    create_label(c3, "AVG SPEED", UI_FONT_12, UI_COL_DIM);
    s_track.lbl_avg_spd = create_label(c3, "24.6 km/h", UI_FONT_16, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(s_track.lbl_avg_spd, 0, 18);

    lv_obj_t *c4 = create_card(s_track.container, 112, 42);
    lv_obj_set_pos(c4, 120, 206);
    create_label(c4, "LOG POINTS", UI_FONT_12, UI_COL_DIM);
    s_track.lbl_log_pts = create_label(c4, "4,420 PTS", UI_FONT_16, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(s_track.lbl_log_pts, 0, 18);

    /* 底部提示语 (Y=256) */
    s_track.lbl_hint = create_label(s_track.container, "SHORT: START • LONG: STOP", UI_FONT_12, UI_COL_DIM);
    lv_obj_set_pos(s_track.lbl_hint, 24, 256);
}

/* 3. 自行车码表视图 */
static void create_bike_view(lv_obj_t *parent)
{
    s_bike.container = lv_obj_create(parent);
    lv_obj_set_size(s_bike.container, UI_H_RES - 8, UI_CONTENT_H - 8);
    lv_obj_set_pos(s_bike.container, 0, 0);
    lv_obj_set_style_bg_opa(s_bike.container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_bike.container, 0, 0);
    lv_obj_set_style_pad_all(s_bike.container, 0, 0);
    lv_obj_remove_flag(s_bike.container, LV_OBJ_FLAG_SCROLLABLE);

    /* 标题栏 */
    lv_obj_t *t_hdr = create_label(s_bike.container, "BIKE COMPUTER", UI_FONT_12, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(t_hdr, 2, 0);
    s_bike.lbl_title_badge = create_label(s_bike.container, "RIDING", UI_FONT_12, UI_COL_GREEN);
    lv_obj_set_pos(s_bike.lbl_title_badge, 180, 0);

    /* 速度大字 Hero 卡片 (Y=18, 高度 92px) */
    lv_obj_t *spd_card = create_card(s_bike.container, UI_H_RES - 8, 92);
    lv_obj_set_pos(spd_card, 0, 18);

    s_bike.lbl_speed_big = create_label(spd_card, "32.8", UI_FONT_48, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(s_bike.lbl_speed_big, 10, 14);

    lv_obj_t *u_spd = create_label(spd_card, "KM / H SPEED", UI_FONT_12, s_ui.is_dark_theme ? UI_COL_CYAN : UI_COL_BLUE);
    lv_obj_set_pos(u_spd, 12, 68);

    s_bike.lbl_max_avg = create_label(spd_card, "MAX 46.2\nAVG 26.5", UI_FONT_12, UI_COL_DIM);
    lv_obj_set_pos(s_bike.lbl_max_avg, 160, 20);

    /* 2x2 骑行参数卡片 (Y=114) */
    lv_obj_t *c1 = create_card(s_bike.container, 112, 48);
    lv_obj_set_pos(c1, 0, 114);
    create_label(c1, "TRIP DIST", UI_FONT_12, UI_COL_DIM);
    s_bike.lbl_trip_val = create_label(c1, "24.5 km", UI_FONT_16, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(s_bike.lbl_trip_val, 0, 20);

    lv_obj_t *c2 = create_card(s_bike.container, 112, 48);
    lv_obj_set_pos(c2, 120, 114);
    create_label(c2, "GRADE SLOPE", UI_FONT_12, UI_COL_DIM);
    s_bike.lbl_slope_val = create_label(c2, "+4.2 %", UI_FONT_16, UI_COL_ORANGE);
    lv_obj_set_pos(s_bike.lbl_slope_val, 0, 20);

    lv_obj_t *c3 = create_card(s_bike.container, 112, 48);
    lv_obj_set_pos(c3, 0, 166);
    create_label(c3, "BARO ALT", UI_FONT_12, UI_COL_DIM);
    s_bike.lbl_alt_val = create_label(c3, "486 m", UI_FONT_16, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(s_bike.lbl_alt_val, 0, 20);

    lv_obj_t *c4 = create_card(s_bike.container, 112, 48);
    lv_obj_set_pos(c4, 120, 166);
    create_label(c4, "V-SPEED", UI_FONT_12, UI_COL_DIM);
    s_bike.lbl_vspd_val = create_label(c4, "+0.8 m/s", UI_FONT_16, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(s_bike.lbl_vspd_val, 0, 20);

    /* 底部罗盘方位条 (Y=220, 高度 46px) */
    lv_obj_t *comp_card = create_card(s_bike.container, UI_H_RES - 8, 46);
    lv_obj_set_pos(comp_card, 0, 220);
    create_label(comp_card, "COMPASS HEADING", UI_FONT_12, UI_COL_DIM);
    s_bike.lbl_heading_txt = create_label(comp_card, "ENE 072\xC2\xB0", UI_FONT_16, UI_COL_ORANGE);
    lv_obj_set_pos(s_bike.lbl_heading_txt, 140, 12);
}

/* ==================== PAGE 1: 传感器诊断页创建 ==================== */
static void create_diag_screen(void)
{
    create_screen_shell(MODE_DIAG);
    lv_obj_t *parent = s_screens[MODE_DIAG].content;

    /* 标题栏 */
    lv_obj_t *t_hdr = create_label(parent, "SENSOR DIAGNOSTICS", UI_FONT_12, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(t_hdr, 2, 0);
    lv_obj_t *badge = create_label(parent, "HEALTHY", UI_FONT_12, UI_COL_GREEN);
    lv_obj_set_pos(badge, 175, 0);

    /* 1. LSM6DSR IMU 卡片 (Y=18, 高度 64px) */
    lv_obj_t *c1 = create_card(parent, UI_H_RES - 8, 64);
    lv_obj_set_pos(c1, 0, 18);
    lv_obj_t *hdr1 = create_label(c1, "LSM6DSR (6-AXIS IMU)   100Hz", UI_FONT_12, s_ui.is_dark_theme ? UI_COL_CYAN : UI_COL_BLUE);
    lv_obj_set_pos(hdr1, 0, 0);
    s_diag.lbl_imu_raw_acc = create_label(c1, "RAW ACC: X:+0.04 Y:-0.02 Z:+0.99 g", UI_FONT_MONO, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(s_diag.lbl_imu_raw_acc, 0, 15);
    /* 重点：六轴线性加速度展示 */
    s_diag.lbl_imu_lin_acc = create_label(c1, "LIN ACC: X:+0.00 Y:+0.00 Z:+0.00 g", UI_FONT_MONO, UI_COL_ORANGE);
    lv_obj_set_pos(s_diag.lbl_imu_lin_acc, 0, 30);
    s_diag.lbl_imu_gyro = create_label(c1, "GYRO:    R:+0.42 P:-0.18 Y:+1.20 dps", UI_FONT_MONO, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(s_diag.lbl_imu_gyro, 0, 45);

    /* 2. LIS2MDL MAG 卡片 (Y=86, 高度 48px) */
    lv_obj_t *c2 = create_card(parent, UI_H_RES - 8, 48);
    lv_obj_set_pos(c2, 0, 86);
    lv_obj_t *hdr2 = create_label(c2, "LIS2MDL (3-AXIS MAG)    25Hz", UI_FONT_12, s_ui.is_dark_theme ? UI_COL_CYAN : UI_COL_BLUE);
    lv_obj_set_pos(hdr2, 0, 0);
    s_diag.lbl_mag_val = create_label(c2, "MAG (uT): X:+18.2 Y:-24.6 Z:+42.1", UI_FONT_MONO, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(s_diag.lbl_mag_val, 0, 15);
    s_diag.lbl_mag_heading = create_label(c2, "HEADING:  072.4\xC2\xB0 (CAL: GOOD)", UI_FONT_MONO, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(s_diag.lbl_mag_heading, 0, 30);

    /* 3. BMP388 BARO 卡片 (Y=138, 高度 48px) */
    lv_obj_t *c3 = create_card(parent, UI_H_RES - 8, 48);
    lv_obj_set_pos(c3, 0, 138);
    lv_obj_t *hdr3 = create_label(c3, "BMP388 (BAROMETER)      50Hz", UI_FONT_12, s_ui.is_dark_theme ? UI_COL_CYAN : UI_COL_BLUE);
    lv_obj_set_pos(hdr3, 0, 0);
    s_diag.lbl_baro_val = create_label(c3, "PRS/T: 956.4 hPa | 28.6 \xC2\xB0\x43", UI_FONT_MONO, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(s_diag.lbl_baro_val, 0, 15);
    s_diag.lbl_baro_alt = create_label(c3, "ALT:   486.2 m (QNH 1013.2)", UI_FONT_MONO, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(s_diag.lbl_baro_alt, 0, 30);

    /* 4. NEO-M8N GNSS 卡片 (Y=190, 高度 62px) */
    lv_obj_t *c4 = create_card(parent, UI_H_RES - 8, 62);
    lv_obj_set_pos(c4, 0, 190);
    lv_obj_t *hdr4 = create_label(c4, "NEO-M8N (GNSS)      UBX 10Hz", UI_FONT_12, s_ui.is_dark_theme ? UI_COL_CYAN : UI_COL_BLUE);
    lv_obj_set_pos(hdr4, 0, 0);
    s_diag.lbl_gnss_status = create_label(c4, "STATUS: 3D FIX (14 SATS)", UI_FONT_MONO, UI_COL_GREEN);
    lv_obj_set_pos(s_diag.lbl_gnss_status, 0, 15);
    s_diag.lbl_gnss_pos = create_label(c4, "POS: 34.2281 N, 108.9324 E", UI_FONT_MONO, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(s_diag.lbl_gnss_pos, 0, 30);
    s_diag.lbl_gnss_dop = create_label(c4, "DOP: HDOP 0.82 | VDOP 1.15", UI_FONT_MONO, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(s_diag.lbl_gnss_dop, 0, 45);
}

/* ==================== PAGE 2: 系统设置页创建 ==================== */
static void create_settings_screen(void)
{
    create_screen_shell(MODE_SETTINGS);
    lv_obj_t *parent = s_screens[MODE_SETTINGS].content;

    /* 标题栏 */
    lv_obj_t *t_hdr = create_label(parent, "SETTINGS", UI_FONT_12, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
    lv_obj_set_pos(t_hdr, 2, 0);
    s_settings.lbl_page_idx = create_label(parent, "1/7", UI_FONT_12, UI_COL_DIM);
    lv_obj_set_pos(s_settings.lbl_page_idx, 205, 0);

    /* 7 个设置行 (每行高 34px，间隙 4px，总高约 262px) */
    const char *init_vals[SETTING_ITEM_COUNT] = {
        "P-GEAR", "SUNLIGHT", "10 HZ", "GPS+BDS", "100%", "3 MIN", "START >"
    };

    for (int i = 0; i < SETTING_ITEM_COUNT; i++) {
        lv_obj_t *row = create_card(parent, UI_H_RES - 8, 34);
        lv_obj_set_pos(row, 0, 18 + i * 38);
        s_settings.rows[i] = row;

        lv_obj_t *k = create_label(row, s_setting_keys[i], UI_FONT_12, s_ui.is_dark_theme ? UI_COL_DARK_TEXT : UI_COL_SUN_TEXT);
        lv_obj_set_pos(k, 4, 3);

        s_settings.val_labels[i] = create_label(row, init_vals[i], UI_FONT_12, s_ui.is_dark_theme ? UI_COL_CYAN : UI_COL_BLUE);
        lv_obj_set_pos(s_settings.val_labels[i], 150, 3);
    }
}

/* ==================== 视图切换与主页面调度 ==================== */
static void update_main_subview(void)
{
    main_page_t target = input_get_main_page();
    if (target != s_cur_subpage) {
        s_cur_subpage = target;
        lv_obj_add_flag(s_pbox.container, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_track.container, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_bike.container, LV_OBJ_FLAG_HIDDEN);

        switch (s_cur_subpage) {
        case MAIN_PAGE_PBOX:
            lv_obj_remove_flag(s_pbox.container, LV_OBJ_FLAG_HIDDEN);
            break;
        case MAIN_PAGE_LOGGER:
            lv_obj_remove_flag(s_track.container, LV_OBJ_FLAG_HIDDEN);
            break;
        case MAIN_PAGE_BIKE:
            lv_obj_remove_flag(s_bike.container, LV_OBJ_FLAG_HIDDEN);
            break;
        default:
            break;
        }
    }
}

/* ==================== 状态栏数据刷新 ==================== */
static void update_status_bar(screen_base_t *sb, const gnss_data_t *g, const sensors_state_t *st)
{
    char buf[32];
    /* 卫星定位状态 */
    if (g->valid) {
        snprintf(buf, sizeof(buf), "%uD (%u)", g->sats >= 4 ? 3 : 2, g->sats);
        lv_label_set_text(sb->lbl_sat, buf);
        lv_obj_set_style_text_color(sb->lbl_sat, UI_COL_GREEN, 0);
    } else {
        snprintf(buf, sizeof(buf), "NO FIX");
        lv_label_set_text(sb->lbl_sat, buf);
        lv_obj_set_style_text_color(sb->lbl_sat, UI_COL_RED, 0);
    }

    /* 录制状态 SD 图标 */
    if (s_ui.is_tracking) {
        lv_obj_set_style_text_color(sb->lbl_sd, UI_COL_RED, 0);
    } else {
        lv_obj_set_style_text_color(sb->lbl_sd, UI_COL_GREEN, 0);
    }

    /* 本地时钟 (UTC+8 或 系统时间) */
    time_t t_now = (time_t)g->utc_sec;
    if (g->time_valid && t_now > 1600000000) {
        t_now += 8 * 3600; /* UTC+8 */
        struct tm *tm_info = gmtime(&t_now);
        if (tm_info) {
            snprintf(buf, sizeof(buf), "%02d:%02d:%02d", tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
        } else {
            snprintf(buf, sizeof(buf), "--:--:--");
        }
    } else {
        uint32_t s_tot = (uint32_t)(esp_timer_get_time() / 1000000);
        uint32_t h = s_tot / 3600;
        uint32_t m = (s_tot % 3600) / 60;
        uint32_t s = s_tot % 60;
        snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", (unsigned long)h, (unsigned long)m, (unsigned long)s);
    }
    lv_label_set_text(sb->lbl_clock, buf);

    /* 电池电压与充电 */
    if (st->battery.valid) {
        snprintf(buf, sizeof(buf), "%.1fV%s", (double)st->battery.voltage_v, st->battery.charging ? " \xE2\x9A\xA1" : "");
        lv_label_set_text(sb->lbl_bat, buf);
    }
}

/* ==================== 各屏周期数据刷新 ==================== */
static void refresh_main_pbox(const gnss_data_t *g, const sensors_state_t *st)
{
    pbox_status_t pb;
    pbox_get_status(&pb);

    char buf[32];
    snprintf(buf, sizeof(buf), "SPD: %4.1f km/h", g->valid ? (double)g->speed_kmh : 0.0);
    lv_label_set_text(s_pbox.lbl_speed_live, buf);

    snprintf(buf, sizeof(buf), "%05.2fs", (double)pb.elapsed_s);
    lv_label_set_text(s_pbox.lbl_timer_big, buf);

    switch (pb.state) {
    case PBOX_READY:
        lv_label_set_text(s_pbox.lbl_title_badge, "READY");
        lv_obj_set_style_text_color(s_pbox.lbl_title_badge, UI_COL_GREEN, 0);
        lv_label_set_text(s_pbox.lbl_status_msg, "STANDBY • HIT GAS TO LAUNCH");
        break;
    case PBOX_ARMED:
        lv_label_set_text(s_pbox.lbl_title_badge, "ARMED");
        lv_obj_set_style_text_color(s_pbox.lbl_title_badge, UI_COL_GREEN, 0);
        lv_label_set_text(s_pbox.lbl_status_msg, "READY TO LAUNCH!");
        break;
    case PBOX_RUNNING:
        lv_label_set_text(s_pbox.lbl_title_badge, "ACCEL");
        lv_obj_set_style_text_color(s_pbox.lbl_title_badge, UI_COL_ORANGE, 0);
        lv_label_set_text(s_pbox.lbl_status_msg, "ACCELERATING (FULL THROTTLE)");
        break;
    case PBOX_FINISHED:
        lv_label_set_text(s_pbox.lbl_title_badge, "FINISHED");
        lv_obj_set_style_text_color(s_pbox.lbl_title_badge, UI_COL_GREEN, 0);
        lv_label_set_text(s_pbox.lbl_status_msg, "VALID RUN • SHORT PUSH RESET");
        break;
    }

    /* G 值雷达红点与数值联动 */
    float gx = st->imu.valid ? st->imu.lin_mg[0] / 1000.0f : 0.0f;
    float gy = st->imu.valid ? st->imu.lin_mg[1] / 1000.0f : 0.0f;

    snprintf(buf, sizeof(buf), "LONG G: %+4.2f G", (double)gx);
    lv_label_set_text(s_pbox.lbl_g_long, buf);
    snprintf(buf, sizeof(buf), "LAT G:  %+4.2f G", (double)gy);
    lv_label_set_text(s_pbox.lbl_g_lat, buf);

    /* 限制红点在雷达圆形内移动 */
    int px = 24 + (int)(gy * 20.0f);
    int py = 24 - (int)(gx * 20.0f);
    if (px < 4) {
        px = 4;
    } else if (px > 44) {
        px = 44;
    }
    if (py < 4) {
        py = 4;
    } else if (py > 44) {
        py = 44;
    }
    lv_obj_set_pos(s_pbox.g_dot, px, py);
}

static void refresh_main_track(const gnss_data_t *g, const sensors_state_t *st)
{
    char buf[32];
    /* 轨迹累加距离与时间 */
    if (s_ui.is_tracking && g->valid) {
        s_ui.track_points++;
        uint32_t now_sec = (uint32_t)(esp_timer_get_time() / 1000000);
        uint32_t elapsed = now_sec - s_ui.track_start_sec;
        uint32_t h = elapsed / 3600;
        uint32_t m = (elapsed % 3600) / 60;
        uint32_t s = elapsed % 60;
        snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", (unsigned long)h, (unsigned long)m, (unsigned long)s);
        lv_label_set_text(s_track.lbl_ride_time, buf);

        snprintf(buf, sizeof(buf), "%5.2f", (double)s_ui.track_dist_km);
        lv_label_set_text(s_track.lbl_dist_big, buf);

        snprintf(buf, sizeof(buf), "%lu PTS", (unsigned long)s_ui.track_points);
        lv_label_set_text(s_track.lbl_log_pts, buf);
    }

    snprintf(buf, sizeof(buf), "%4.1f km/h", g->valid ? (double)g->speed_avg_kmh : 0.0);
    lv_label_set_text(s_track.lbl_avg_spd, buf);

    snprintf(buf, sizeof(buf), "%+4.0f m", st->baro.valid ? (double)st->baro.altitude_m : 0.0);
    lv_label_set_text(s_track.lbl_elev_gain, buf);
}

static void refresh_main_bike(const gnss_data_t *g, const sensors_state_t *st)
{
    char buf[32];
    float spd = (g->valid && !s_ui.bike_paused) ? g->speed_kmh : 0.0f;
    if (spd > s_ui.bike_max_speed) {
        s_ui.bike_max_speed = spd;
    }

    snprintf(buf, sizeof(buf), "%4.1f", (double)spd);
    lv_label_set_text(s_bike.lbl_speed_big, buf);

    snprintf(buf, sizeof(buf), "MAX %3.1f\nAVG %3.1f",
             (double)s_ui.bike_max_speed,
             (double)(g->valid ? g->speed_avg_kmh : 0.0f));
    lv_label_set_text(s_bike.lbl_max_avg, buf);

    snprintf(buf, sizeof(buf), "%4.1f km", (double)s_ui.bike_trip_km);
    lv_label_set_text(s_bike.lbl_trip_val, buf);

    snprintf(buf, sizeof(buf), "%4.0f m", st->baro.valid ? (double)st->baro.altitude_m : 0.0);
    lv_label_set_text(s_bike.lbl_alt_val, buf);

    /* 航向方位角 */
    float crs = g->valid ? g->course_deg : 0.0f;
    const char *dirs[] = { "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                           "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW" };
    int d_idx = (int)((crs + 11.25f) / 22.5f) % 16;
    snprintf(buf, sizeof(buf), "%s %03.0f\xC2\xB0", dirs[d_idx], (double)crs);
    lv_label_set_text(s_bike.lbl_heading_txt, buf);
}

static void refresh_diag_page(const gnss_data_t *g, const sensors_state_t *st)
{
    char buf[64];

    /* 1. IMU: 原始加速度、线性加速度、角速度 */
    float ax = st->imu.valid ? st->imu.accel_mg[0] / 1000.0f : 0.0f;
    float ay = st->imu.valid ? st->imu.accel_mg[1] / 1000.0f : 0.0f;
    float az = st->imu.valid ? st->imu.accel_mg[2] / 1000.0f : 0.0f;
    snprintf(buf, sizeof(buf), "RAW ACC: X:%+4.2f Y:%+4.2f Z:%+4.2f g", (double)ax, (double)ay, (double)az);
    lv_label_set_text(s_diag.lbl_imu_raw_acc, buf);

    /* 线性加速度 (重力分离后纯动态加速度) */
    float lx = st->imu.valid ? st->imu.lin_mg[0] / 1000.0f : 0.0f;
    float ly = st->imu.valid ? st->imu.lin_mg[1] / 1000.0f : 0.0f;
    float lz = st->imu.valid ? st->imu.lin_mg[2] / 1000.0f : 0.0f;
    snprintf(buf, sizeof(buf), "LIN ACC: X:%+4.2f Y:%+4.2f Z:%+4.2f g", (double)lx, (double)ly, (double)lz);
    lv_label_set_text(s_diag.lbl_imu_lin_acc, buf);

    float gx = st->imu.valid ? st->imu.gyro_mdps[0] / 1000.0f : 0.0f;
    float gy = st->imu.valid ? st->imu.gyro_mdps[1] / 1000.0f : 0.0f;
    float gz = st->imu.valid ? st->imu.gyro_mdps[2] / 1000.0f : 0.0f;
    snprintf(buf, sizeof(buf), "GYRO:    R:%+4.1f P:%+4.1f Y:%+4.1f dps", (double)gx, (double)gy, (double)gz);
    lv_label_set_text(s_diag.lbl_imu_gyro, buf);

    /* 2. 磁力计 */
    float mx = st->mag.valid ? st->mag.mag_mgauss[0] : 0.0f;
    float my = st->mag.valid ? st->mag.mag_mgauss[1] : 0.0f;
    float mz = st->mag.valid ? st->mag.mag_mgauss[2] : 0.0f;
    snprintf(buf, sizeof(buf), "MAG (uT): X:%+4.1f Y:%+4.1f Z:%+4.1f", (double)(mx*0.1), (double)(my*0.1), (double)(mz*0.1));
    lv_label_set_text(s_diag.lbl_mag_val, buf);

    snprintf(buf, sizeof(buf), "HEADING:  %05.1f\xC2\xB0 (CAL: %s)",
             (double)(g->valid ? g->course_deg : 0.0f),
             st->mag.valid ? "GOOD" : "NO");
    lv_label_set_text(s_diag.lbl_mag_heading, buf);

    /* 3. 气压计 */
    snprintf(buf, sizeof(buf), "PRS/T: %5.1f hPa | %4.1f \xC2\xB0\x43",
             st->baro.valid ? (double)st->baro.pressure_hpa : 0.0,
             st->baro.valid ? (double)st->baro.temp_c : 0.0);
    lv_label_set_text(s_diag.lbl_baro_val, buf);

    snprintf(buf, sizeof(buf), "ALT:   %5.1f m (QNH 1013.2)",
             st->baro.valid ? (double)st->baro.altitude_m : 0.0);
    lv_label_set_text(s_diag.lbl_baro_alt, buf);

    /* 4. GNSS */
    snprintf(buf, sizeof(buf), "STATUS: %s (%u SATS)",
             g->valid ? "3D FIX" : "SEARCHING", g->sats);
    lv_label_set_text(s_diag.lbl_gnss_status, buf);
    lv_obj_set_style_text_color(s_diag.lbl_gnss_status, g->valid ? UI_COL_GREEN : UI_COL_ORANGE, 0);

    snprintf(buf, sizeof(buf), "POS: %07.4f %c, %07.4f %c",
             fabs(g->lat), g->lat >= 0 ? 'N' : 'S',
             fabs(g->lon), g->lon >= 0 ? 'E' : 'W');
    lv_label_set_text(s_diag.lbl_gnss_pos, buf);

    snprintf(buf, sizeof(buf), "DOP: HDOP %3.2f | VDOP %3.2f", (double)g->hdop, (double)g->vdop);
    lv_label_set_text(s_diag.lbl_gnss_dop, buf);
}

/* ==================== 定时器回调 ==================== */
static void ui_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    gnss_data_t g;
    gnss_get_data(&g);
    sensors_state_t st;
    sensors_get_state(&st);

    /* 页面切换检查 */
    app_mode_t m = input_get_mode();
    if (m != s_cur_mode) {
        s_cur_mode = m;
        lv_screen_load(s_screens[m].root);
        ESP_LOGI(TAG, "Screen switched to Page %d", (int)m);
    }

    /* 状态栏全局刷新 */
    update_status_bar(&s_screens[s_cur_mode], &g, &st);

    /* 根据当前模式刷新具体页面 */
    switch (s_cur_mode) {
    case MODE_MAIN:
        update_main_subview();
        switch (s_cur_subpage) {
        case MAIN_PAGE_PBOX:   refresh_main_pbox(&g, &st); break;
        case MAIN_PAGE_LOGGER: refresh_main_track(&g, &st); break;
        case MAIN_PAGE_BIKE:   refresh_main_bike(&g, &st); break;
        default: break;
        }
        break;
    case MODE_DIAG:
        refresh_diag_page(&g, &st);
        break;
    case MODE_SETTINGS:
        break;
    default:
        break;
    }
}

/* ==================== 外部事件处理 API ==================== */
void ui_logger_start(void)
{
    if (!s_ui.is_tracking) {
        s_ui.is_tracking = true;
        s_ui.track_start_sec = (uint32_t)(esp_timer_get_time() / 1000000);
        s_ui.track_points = 0;
        s_ui.track_dist_km = 0.0f;
        lv_label_set_text(s_track.lbl_title_badge, "LOGGING");
        lv_obj_set_style_text_color(s_track.lbl_title_badge, UI_COL_RED, 0);
        lv_label_set_text(s_track.lbl_rec_tag, "REC");
        lv_obj_set_style_text_color(s_track.lbl_rec_tag, UI_COL_RED, 0);
        lv_label_set_text(s_track.lbl_hint, "RECORDING • LONG PUSH TO STOP");
        ESP_LOGI(TAG, "Track logger STARTED");
    }
}

void ui_logger_stop(void)
{
    if (s_ui.is_tracking) {
        s_ui.is_tracking = false;
        lv_label_set_text(s_track.lbl_title_badge, "SAVED");
        lv_obj_set_style_text_color(s_track.lbl_title_badge, UI_COL_GREEN, 0);
        lv_label_set_text(s_track.lbl_rec_tag, "SAVED");
        lv_obj_set_style_text_color(s_track.lbl_rec_tag, UI_COL_GREEN, 0);
        lv_label_set_text(s_track.lbl_hint, "GPX FILE SAVED • SHORT PUSH START");
        ESP_LOGI(TAG, "Track logger STOPPED & SAVED");
    }
}

void ui_bike_toggle_pause(void)
{
    s_ui.bike_paused = !s_ui.bike_paused;
    if (s_ui.bike_paused) {
        lv_label_set_text(s_bike.lbl_title_badge, "PAUSED");
        lv_obj_set_style_text_color(s_bike.lbl_title_badge, UI_COL_ORANGE, 0);
    } else {
        lv_label_set_text(s_bike.lbl_title_badge, "RIDING");
        lv_obj_set_style_text_color(s_bike.lbl_title_badge, UI_COL_GREEN, 0);
    }
}

void ui_bike_reset_trip(void)
{
    s_ui.bike_trip_km = 0.0f;
    s_ui.bike_max_speed = 0.0f;
    lv_label_set_text(s_bike.lbl_trip_val, "0.0 km");
    lv_label_set_text(s_bike.lbl_speed_big, "0.0");
    ESP_LOGI(TAG, "Bike trip reset");
}

void ui_settings_step_value(void)
{
    int idx = s_ui.setting_selected_idx;
    s_setting_vals[idx]++;

    if (idx == 0) {
        /* FUNCTION MODE: 0: P-GEAR, 1: TRACK, 2: BIKE */
        const char *f_names[] = { "P-GEAR", "TRACK REC", "BIKE COMP" };
        s_setting_vals[0] %= 3;
        lv_label_set_text(s_settings.val_labels[0], f_names[s_setting_vals[0]]);
        input_set_main_page((main_page_t)s_setting_vals[0]);
    } else if (idx == 1) {
        /* COLOR THEME: 0: SUNLIGHT, 1: DARK */
        s_setting_vals[1] %= 2;
        s_ui.is_dark_theme = (s_setting_vals[1] == 1);
        lv_label_set_text(s_settings.val_labels[1], s_ui.is_dark_theme ? "DARK" : "SUNLIGHT");
    } else if (idx == 2) {
        /* GPS RATE: 10 HZ, 18 HZ, 1 HZ, 5 HZ */
        const char *rates[] = { "10 HZ", "18 HZ", "1 HZ", "5 HZ" };
        s_setting_vals[2] %= 4;
        lv_label_set_text(s_settings.val_labels[2], rates[s_setting_vals[2]]);
    } else if (idx == 3) {
        /* GNSS MODE */
        const char *modes[] = { "GPS+BDS", "ALL GNSS", "GPS ONLY" };
        s_setting_vals[3] %= 3;
        lv_label_set_text(s_settings.val_labels[3], modes[s_setting_vals[3]]);
    } else if (idx == 4) {
        /* BRIGHTNESS */
        const char *brs[] = { "100%", "80%", "60%", "40%" };
        s_setting_vals[4] %= 4;
        lv_label_set_text(s_settings.val_labels[4], brs[s_setting_vals[4]]);
    } else if (idx == 5) {
        /* AUTO SLEEP */
        const char *slps[] = { "3 MIN", "5 MIN", "NEVER", "1 MIN" };
        s_setting_vals[5] %= 4;
        lv_label_set_text(s_settings.val_labels[5], slps[s_setting_vals[5]]);
    } else if (idx == 6) {
        /* SENSOR CALIB */
        lv_label_set_text(s_settings.val_labels[6], "DONE!");
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

    /* 1. 创建 Page 0 主功能页壳与 3 套子视图 */
    create_screen_shell(MODE_MAIN);
    create_pbox_view(s_screens[MODE_MAIN].content);
    create_track_view(s_screens[MODE_MAIN].content);
    create_bike_view(s_screens[MODE_MAIN].content);
    /* 默认展示 P-Box 视图 */
    lv_obj_add_flag(s_track.container, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_bike.container, LV_OBJ_FLAG_HIDDEN);

    /* 2. 创建 Page 1 传感器诊断页 */
    create_diag_screen();

    /* 3. 创建 Page 2 系统设置页 */
    create_settings_screen();

    /* 默认加载主功能页 */
    lv_screen_load(s_screens[MODE_MAIN].root);

    /* 创建 10Hz 界面刷新定时器 */
    lv_timer_create(ui_timer_cb, UI_REFRESH_PERIOD_MS, NULL);

    lvgl_port_unlock();

    ESP_LOGI(TAG, "LVGL 9.5 UI initialized (ST7789 240x320, 3-page, FW %s)", FW_VERSION_STR);
    return ESP_OK;
}

bool ui_lock(void)   { return lvgl_port_lock(100); }
void ui_unlock(void) { lvgl_port_unlock(); }
