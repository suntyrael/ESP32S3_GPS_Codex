/*
 * lcd_driver.c - LCD 显示驱动（esp_lcd + ST7789 240x320，SPI3 + DMA 双缓冲）
 * 引脚：RST=4 SCK=5 DC=6 CS=7 MOSI=8 BL=9（config.h）
 * 显示规格（README §3.2）：ST7789 240x320 竖屏，旋转 180°；背光 Q3 + LEDC 2kHz，默认 50%
 */
#include "lcd_driver.h"
#include "config.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "lcd_driver";

static esp_lcd_panel_io_handle_t s_io = NULL;
static esp_lcd_panel_handle_t s_panel = NULL;

esp_err_t lcd_driver_init(void)
{
    /* ---- SPI 总线：SPI3，SCK=5 / MOSI=8，80 MHz ---- */
    spi_bus_config_t bus_cfg = {
        .sclk_io_num = PIN_LCD_SCK,
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * 2,   /* 全屏一帧，DMA */
    };
    esp_err_t ret = spi_bus_initialize(LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* ---- 面板 IO（SPI 命令/数据） ---- */
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = PIN_LCD_DC,
        .cs_gpio_num = PIN_LCD_CS,
        .pclk_hz = LCD_SPI_CLK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST, &io_cfg, &s_io);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_new_panel_io_spi failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* ---- ST7789 面板 ---- */
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = PIN_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ret = esp_lcd_new_panel_st7789(s_io, &panel_cfg, &s_panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_new_panel_st7789 failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, LCD_INVERT_COLOR));
#if LCD_MIRROR_X || LCD_MIRROR_Y || LCD_SWAP_XY
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, LCD_MIRROR_X, LCD_MIRROR_Y));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(s_panel, LCD_SWAP_XY));
#endif
#if LCD_SET_GAP
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(s_panel, LCD_GAP_X, LCD_GAP_Y));
#endif
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));

    /* ---- 背光：GPIO9 LEDC 2kHz PWM，默认 50% ---- */
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = LCD_BL_PWM_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ledc_timer_config failed: %s", esp_err_to_name(ret));
    }
    ledc_channel_config_t ledc_chan = {
        .gpio_num = PIN_LCD_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ret = ledc_channel_config(&ledc_chan);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ledc_channel_config failed: %s", esp_err_to_name(ret));
    }
    lcd_backlight_set(LCD_BL_DEFAULT_PERCENT);

    ESP_LOGI(TAG, "ST7789 %ux%u ready @SPI3 %luHz", LCD_H_RES, LCD_V_RES, (unsigned long)LCD_SPI_CLK_HZ);
    return ESP_OK;
}

esp_lcd_panel_handle_t lcd_driver_get_panel(void) { return s_panel; }
esp_lcd_panel_io_handle_t lcd_driver_get_io(void) { return s_io; }

void lcd_backlight_set(uint8_t percent)
{
    if (percent > 100) {
        percent = 100;
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)percent * 255 / 100);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}
