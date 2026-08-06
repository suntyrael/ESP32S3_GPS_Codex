/*
 * lcd_driver.h - LCD 显示驱动（esp_lcd + ST7789，SPI3）
 * 引脚：RST=4 SCK=5 DC=6 CS=7 MOSI=8 BL=9（config.h）
 * 背光：Q3 MOSFET + LEDC 2kHz PWM
 */
#pragma once

#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"

/**
 * @brief 初始化 SPI 总线 + ST7789 面板 + 背光
 * @return ESP_OK 成功
 */
esp_err_t lcd_driver_init(void);

/** @brief 获取面板句柄（供 LVGL port 使用） */
esp_lcd_panel_handle_t lcd_driver_get_panel(void);

/** @brief 获取面板 IO 句柄（供 LVGL port 使用） */
esp_lcd_panel_io_handle_t lcd_driver_get_io(void);

/** @brief 设置背光亮度 0-100% */
void lcd_backlight_set(uint8_t percent);
