/*
 * ui_common.h - UI 布局常量（README §7 UI 规格）
 * 约束 C-06：UI 层禁止魔数，全部常量收敛于此
 */
#pragma once

/* ==================== 屏幕 ==================== */
#define UI_H_RES                240
#define UI_V_RES                320

/* ==================== 布局 ==================== */
#define UI_STATUS_H             20          /* 状态栏高度 px */
#define UI_PAD_S                5           /* 小间距 */
#define UI_PAD_M                10          /* 中间距 */
#define UI_PAD_L                15          /* 大间距 */

/* ==================== 字号（LVGL 内置蒙塞拉特） ==================== */
#define UI_FONT_SMALL           12          /* 状态栏 */
#define UI_FONT_MEDIUM          16          /* 数据标签 */
#define UI_FONT_LARGE           24          /* 区块标题 */
#define UI_FONT_XL              32          /* 主数值 */

/* ==================== 颜色 ==================== */
#define UI_COL_BG               lv_color_black()
#define UI_COL_TEXT             lv_color_white()
#define UI_COL_SUB              lv_color_hex(0x808080)
#define UI_COL_WARN             lv_color_hex(0xFFA500)
#define UI_COL_REC              lv_color_hex(0xFF0000)
#define UI_COL_GPS_OK           lv_color_hex(0x00FF00)
#define UI_COL_GPS_NO           lv_color_hex(0xFF0000)
#define UI_COL_YELLOW           lv_color_hex(0xFFFF00)   /* 明黄：诊断页其余信息 */

/* ==================== 刷新周期 ==================== */
#define UI_REFRESH_PERIOD_MS    200         /* UI 数据刷新周期 */
