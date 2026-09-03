/*
 * ui.h - LVGL 界面（阶段 2：自检/传感器数据界面 + 状态栏）
 * 约束 C-09：UI 只在 LVGL 任务内更新（lvgl_port_init 自带任务 + lv_timer 周期刷新）
 */
#pragma once

#include "esp_err.h"

/** @brief 初始化 LVGL port（自带任务/timer）并添加 ST7789 显示，创建界面
 * @return ESP_OK 成功
 */
esp_err_t ui_init(void);

/** @brief 获取 LVGL 刷新 mutex（跨任务更新 UI 前调用，超时 100ms） */
bool ui_lock(void);

/** @brief 释放 LVGL mutex */
void ui_unlock(void);

/** @brief 轨迹记录：短按开启记录 */
void ui_logger_start(void);

/** @brief 轨迹记录：长按停止记录 */
void ui_logger_stop(void);

/** @brief 码表：短按切换暂停/继续 */
void ui_bike_toggle_pause(void);

/** @brief 码表：长按单次里程与极速清零 */
void ui_bike_reset_trip(void);

/** @brief 设置页：短按循环切换配置条目值 */
void ui_settings_step_value(void);
