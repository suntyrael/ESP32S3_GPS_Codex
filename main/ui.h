/*
 * ui.h - LVGL 界面（阶段 2：自检/传感器数据界面 + 状态栏）
 * 约束 C-09：UI 只在 LVGL 任务内更新（lvgl_port_init 自带任务 + lv_timer 周期刷新）
 */
#pragma once

#include "esp_err.h"
#include "input.h"

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

/** @brief 传感器诊断页：切换健康页 / 调试页（NMEA LOG） */
void ui_diag_toggle_gnss_log(void);

/* ==================== Page 0: P-Gear 成绩交互 ==================== */

/** @brief 双击：清空当前成绩（历史记录保留；由输入层连击窗口结算后调用） */
void ui_pbox_on_double(void);

/** @brief 三击：打开历届成绩清单（同时撤销待执行的双击清零） */
void ui_pbox_on_triple(void);

/** @brief 清单内波轮：上下移动选中记录行
 *  @return true 表示事件已被清单消费（调用方不得再切大页面） */
bool ui_pbox_history_enc(int dir);

/** @brief 清单内长按：退出清单
 *  @return true 表示已关闭清单 */
bool ui_pbox_history_long(void);

/** @brief 清单是否处于打开状态 */
bool ui_pbox_history_is_open(void);

/**
 * @brief 诊断页事件处理（由 app_task 调用）
 *  - 双击：健康页 ↔ 调试页（进入时默认 NMEA LOG）
 *  - 调试页滚轮：NMEA LOG ↔ SAT INFO
 *  - 调试页长按：返回健康页（不返回主页面）
 * @return true 表示事件已被诊断页消费，调用方不得再切大页面
 */
bool ui_diag_handle_event(input_event_t ev);

/** @brief 设置页子状态机枚举 */
typedef enum {
    SETTING_STATE_PAGE = 0,     /* 页面浏览态（波轮切换大页，短按进入光标态） */
    SETTING_STATE_CURSOR,       /* 设置项光标选择态（波轮上下移动光标，短按进入或切换） */
    SETTING_STATE_EDIT_FUNC,    /* Function Mode 编辑态（波轮切换子功能，短按确认退出） */
    SETTING_STATE_CALIB,        /* 传感器校准流程态 */
} setting_substate_t;

/** @brief 获取当前设置页子状态 */
setting_substate_t ui_settings_get_substate(void);

/** @brief 设置页处理波轮旋转 (dir: +1 下/CW, -1 上/CCW) */
void ui_settings_handle_enc(int dir);

/** @brief 设置页处理短按按键 */
void ui_settings_handle_short(void);

/** @brief 设置页处理长按按键（返回 true 表示需要退回 MODE_MAIN，返回 false 表示仅内部取消） */
bool ui_settings_handle_long(void);

/** @brief 检查 RTC 自动同步是否开启 */
bool ui_settings_is_rtc_auto_sync_enabled(void);

/** @brief 检查高度自动校准是否开启 */
bool ui_settings_is_alt_auto_calib_enabled(void);

/** @brief 设置页：短按循环切换配置条目值（兼容旧调用） */
void ui_settings_step_value(void);
