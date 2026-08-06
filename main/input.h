/*
 * input.h - 输入层（编码器 + 主按键）与模式管理
 * 编码器 ENC_A=1 / ENC_B=3（PCNT 正交）；主按键 KEY_MAIN=2
 * 事件：旋转切模式；短按=确认；中按(500ms)=记录开关；长按(2s)=设置；双击(≤400ms)=预留
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    MODE_BIKE_COMPUTER = 0,
    MODE_GPS_LOGGER,
    MODE_PBOX,
    MODE_GNSS_INFO,
    MODE_SETTINGS,
    MODE_MAX
} app_mode_t;

typedef enum {
    INPUT_EV_MODE_NEXT,      /* 编码器右旋：下一模式 */
    INPUT_EV_MODE_PREV,      /* 编码器左旋：上一模式 */
    INPUT_EV_KEY_SHORT,      /* 短按：确认/执行 */
    INPUT_EV_KEY_MIDDLE,     /* 中按：开始/停止轨迹记录 */
    INPUT_EV_KEY_LONG,       /* 长按：进入/退出设置 */
    INPUT_EV_KEY_DOUBLE,     /* 双击：预留 */
} input_event_t;

/** @brief 初始化 PCNT 编码器 + 按键，创建输入任务 */
esp_err_t input_init(void);

/** @brief 非阻塞取输入事件（无事件返回 false） */
bool input_get_event(input_event_t *ev);

/** @brief 当前模式 */
app_mode_t input_get_mode(void);
