/*
 * ui_common.h - UI 布局常量（README §7 UI 规格与 HTML 确认设计）
 * 约束 C-06：UI 层禁止魔数，全部常量收敛于此
 */
#pragma once

#include "lvgl.h"

/* ==================== 屏幕物理尺寸 ==================== */
#define UI_H_RES                240
#define UI_V_RES                320

/* ==================== 布局高度与间距 ==================== */
#define UI_STATUS_H             20          /* 状态栏高度 px */
#define UI_NAV_H                16          /* 底部导航栏高度 px */
#define UI_CONTENT_H            (UI_V_RES - UI_STATUS_H - UI_NAV_H) /* 284px 视口高度 */

#define UI_PAD_NONE             0
#define UI_PAD_XS               2           /* 超小间距 */
#define UI_PAD_S                4           /* 小间距 */
#define UI_PAD_M                6           /* 中间距 */
#define UI_PAD_L                10          /* 大间距 */

#define UI_RADIUS_S             3
#define UI_RADIUS_M             5
#define UI_BORDER_WIDTH         1

/* ==================== 颜色定义 (Sunlight & Dark) ==================== */
/* Sunlight 日光高对比度模式 */
#define UI_COL_SUN_BG           lv_color_hex(0xF8FAFC)
#define UI_COL_SUN_CARD         lv_color_hex(0xFFFFFF)
#define UI_COL_SUN_BORDER       lv_color_hex(0x94A3B8)
#define UI_COL_SUN_TEXT         lv_color_hex(0x000000)
#define UI_COL_SUN_SUB          lv_color_hex(0x334155)
#define UI_COL_SUN_DIM          lv_color_hex(0x64748B)
#define UI_COL_SUN_ACCENT       lv_color_hex(0x0044CC)

/* Dark 暗夜纯黑模式 */
#define UI_COL_DARK_BG          lv_color_hex(0x000000)
#define UI_COL_DARK_CARD        lv_color_hex(0x0B0F17)
#define UI_COL_DARK_BORDER      lv_color_hex(0x232C3D)
#define UI_COL_DARK_TEXT        lv_color_hex(0xFFFFFF)
#define UI_COL_DARK_SUB         lv_color_hex(0xCBD5E1)
#define UI_COL_DARK_DIM         lv_color_hex(0x718096)
#define UI_COL_DARK_ACCENT      lv_color_hex(0x00D4FF)

/* 功能强调色 */
#define UI_COL_GREEN            lv_color_hex(0x15803D)   /* 3D FIX / 就绪 / 健康 */
#define UI_COL_RED              lv_color_hex(0xB91C1C)   /* REC 录制 / 警告 */
#define UI_COL_ORANGE           lv_color_hex(0xC2410C)   /* 加速中 / 线性加速度高亮 / 航向 */
#define UI_COL_BLUE             lv_color_hex(0x0044CC)   /* 强调蓝 */
#define UI_COL_CYAN             lv_color_hex(0x00D4FF)   /* 强调青（暗夜） */

/* 默认背景与文字（Sunlight 户外强光高对比度为默认） */
#define UI_COL_BG               UI_COL_SUN_BG
#define UI_COL_TEXT             UI_COL_SUN_TEXT
#define UI_COL_SUB              UI_COL_SUN_SUB
#define UI_COL_DIM              UI_COL_SUN_DIM
#define UI_COL_CARD             UI_COL_SUN_CARD
#define UI_COL_BORDER           UI_COL_SUN_BORDER

/* ==================== Google Fonts 字体库声明 (与 HTML UI 完全一致) ==================== */
LV_FONT_DECLARE(font_chakra_petch_48);  /* 48px: P-Box计时、码表速度、轨迹里程超大数字 */
LV_FONT_DECLARE(font_chakra_petch_16);  /* 16px: 仪表卡片关键数值 */
LV_FONT_DECLARE(font_oswald_14);        /* 14px: 卡片标题、模式名称 */
LV_FONT_DECLARE(font_oswald_12);        /* 12px: 状态栏标签、辅助说明 */
LV_FONT_DECLARE(font_roboto_mono_12);   /* 12px: 诊断页传感器等宽数据、时钟 */

#define UI_FONT_12              (&font_oswald_12)
#define UI_FONT_14              (&font_oswald_14)
#define UI_FONT_16              (&font_chakra_petch_16)
#define UI_FONT_48              (&font_chakra_petch_48)
#define UI_FONT_MONO            (&font_roboto_mono_12)

/* ==================== 刷新周期 ==================== */
#define UI_REFRESH_PERIOD_MS    100         /* UI 数据刷新周期 10Hz */
