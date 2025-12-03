#ifndef COMPONENT_UI_H
#define COMPONENT_UI_H

/**
 * @file ui.h
 * @brief LVGL + LovyanGFX 图形界面入口，包含主菜单、Bike/GPX/P-GEAR 页面与设置界面。
 */

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 UI 子系统。
 */
esp_err_t ui_init(void);

/**
 * @brief 根据输入事件切换界面。
 */
esp_err_t ui_handle_input(int input_event);

/**
 * @brief 刷新 UI 状态。
 */
esp_err_t ui_update(bool has_fix);

#ifdef __cplusplus
}
#endif

#endif // COMPONENT_UI_H
