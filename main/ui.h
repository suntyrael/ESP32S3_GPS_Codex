/*
 * ui.h - LVGL 界面（阶段 2：自检/传感器数据界面 + 状态栏）
 * 约束 C-09：UI 只在 LVGL 任务内更新（lvgl_port_init 自带任务 + lv_timer 周期刷新）
 */
#pragma once

#include "esp_err.h"

/**
 * @brief 初始化 LVGL port（自带任务/timer）并添加 ST7789 显示，创建界面
 * @return ESP_OK 成功
 */
esp_err_t ui_init(void);

/** @brief 获取 LVGL 刷新 mutex（跨任务更新 UI 前调用，超时 100ms） */
bool ui_lock(void);

/** @brief 释放 LVGL mutex */
void ui_unlock(void);
