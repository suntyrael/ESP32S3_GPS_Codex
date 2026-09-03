/*
 * input.h - 输入层（编码器 + 主按键）与页面/主页面类型管理
 * 页面：主页面(码表/GPS记录/P-Box 三选一) / 诊断 / 设置 —— 3 页循环
 * 编码器：页面切换；短按：设置页内切主页面类型 / P-Box 确认；中按：记录开关；长按：预留
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    MODE_MAIN = 0,      /* 主页面（类型见 main_page_t，设置中更改） */
    MODE_DIAG,          /* 诊断：GNSS + 三传感器 + 电池 */
    MODE_SETTINGS,      /* 设置 */
    MODE_MAX
} app_mode_t;

typedef enum {
    MAIN_PAGE_PBOX = 0,     /* P-GEAR / P-Box 性能测试 */
    MAIN_PAGE_LOGGER,       /* TRACK REC 轨迹记录仪 */
    MAIN_PAGE_BIKE,         /* BIKE COMP 自行车码表 */
    MAIN_PAGE_MAX
} main_page_t;

typedef enum {
    INPUT_EV_MODE_NEXT,      /* 编码器右旋：下一页面 */
    INPUT_EV_MODE_PREV,      /* 编码器左旋：上一页面 */
    INPUT_EV_KEY_SHORT,      /* 短按：设置页切主页面类型 / P-Box 确认 */
    INPUT_EV_KEY_MIDDLE,     /* 中按：开始/停止轨迹记录 */
    INPUT_EV_KEY_LONG,       /* 长按：预留 */
    INPUT_EV_KEY_DOUBLE,     /* 双击：预留 */
} input_event_t;

/** @brief 初始化 PCNT 编码器 + 按键，创建输入任务 */
esp_err_t input_init(void);

/** @brief 非阻塞取输入事件（无事件返回 false） */
bool input_get_event(input_event_t *ev);

/** @brief 当前页面 */
app_mode_t input_get_mode(void);
void input_set_mode(app_mode_t m);

/** @brief 主页面类型（设置中更改；暂存 RAM，阶段 6 落 NVS） */
main_page_t input_get_main_page(void);
void input_set_main_page(main_page_t p);
