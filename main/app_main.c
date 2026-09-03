/*
 * app_main.c - 入口（阶段 2：传感器 + LCD + LVGL UI）
 * 流程：NVS -> 传感器汇聚层 -> LCD/LVGL -> sensor_task / diagnostics_task
 */
#include "config.h"
#include "sensors.h"
#include "diagnostics.h"
#include "lcd_driver.h"
#include "ui.h"
#include "gnss.h"
#include "input.h"
#include "pbox.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_idf_version.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "app_main";

static void sensor_task(void *arg)
{
    for (;;) {
        sensors_update();
        vTaskDelay(pdMS_TO_TICKS(SENSOR_LOOP_MS));
    }
    (void)arg;
}

/* 应用任务：输入事件分发 + P-Box 周期更新（50ms） */
static void app_task(void *arg)
{
    (void)arg;
    for (;;) {
        input_event_t ev;
        while (input_get_event(&ev)) {
            switch (ev) {
            case INPUT_EV_KEY_SHORT:
                if (input_get_mode() == MODE_SETTINGS) {
                    /* 设置页：短按步进修改选中的设置项 */
                    ui_settings_step_value();
                } else if (input_get_mode() == MODE_MAIN) {
                    main_page_t mp = input_get_main_page();
                    if (mp == MAIN_PAGE_PBOX) {
                        pbox_arm();     /* P-Box：复位或就绪 */
                    } else if (mp == MAIN_PAGE_LOGGER) {
                        ui_logger_start(); /* 轨迹记录：开启短按 */
                    } else if (mp == MAIN_PAGE_BIKE) {
                        ui_bike_toggle_pause(); /* 码表：短按暂停/继续 */
                    }
                }
                break;
            case INPUT_EV_KEY_LONG:
                if (input_get_mode() == MODE_MAIN) {
                    main_page_t mp = input_get_main_page();
                    if (mp == MAIN_PAGE_LOGGER) {
                        ui_logger_stop(); /* 轨迹记录：长按停止 */
                    } else if (mp == MAIN_PAGE_BIKE) {
                        ui_bike_reset_trip(); /* 码表：长按清零 */
                    }
                } else {
                    /* 诊断页或设置页：长按返回 Page 0 (主页) */
                    input_set_mode(MODE_MAIN);
                    ESP_LOGI(TAG, "long press: return to MODE_MAIN");
                }
                break;
            default:
                break;          /* 页面切换由 UI 轮询 input_get_mode 处理 */
            }
        }
        /* P-Box 输入：GNSS 速度 + IMU X 轴线性加速度 */
        gnss_data_t g;
        gnss_get_data(&g);
        sensors_state_t st;
        sensors_get_state(&st);
        float acc_x = st.imu.valid ? st.imu.lin_mg[0] / 1000.0f : 0.0f;
        pbox_update(g.valid ? g.speed_kmh : 0.0f, acc_x);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static esp_err_t nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

void app_main(void)
{
    esp_chip_info_t chip;
    esp_chip_info(&chip);

    /* 串口醒目打印固件版本号与平台信息 */
    ESP_LOGI(TAG, "==========================================================");
    ESP_LOGI(TAG, "  ESP32-S3 GPS Performance Analyzer & Outdoor Tracker");
    ESP_LOGI(TAG, "  Firmware Version : %s", FW_VERSION_STR);
    ESP_LOGI(TAG, "  Build Timestamp  : %s %s", __DATE__, __TIME__);
    ESP_LOGI(TAG, "  ESP-IDF Version  : %s", esp_get_idf_version());
    ESP_LOGI(TAG, "  Target Hardware  : ESP32-S3FH4R2 (%u Cores, 4MB Flash, 2MB PSRAM)", (unsigned)chip.cores);
    ESP_LOGI(TAG, "  Display Size     : ST7789 %dx%d Portrait", LCD_H_RES, LCD_V_RES);
    ESP_LOGI(TAG, "==========================================================");

    ESP_ERROR_CHECK(nvs_init());

    esp_err_t ret = sensors_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "sensors_init 失败: %s", esp_err_to_name(ret));
    }

    /* GNSS（阶段 3）：LDO 使能 + UART1 + 解析任务；失败降级 */
    if (gnss_init() != ESP_OK) {
        ESP_LOGE(TAG, "GNSS 初始化失败，降级 N/A");
    }

    /* 输入 + P-Box（阶段 4） */
    pbox_init();
    if (input_init() == ESP_OK) {
        xTaskCreate(app_task, "app_task", TASK_STACK_INPUT, NULL, TASK_PRIO_INPUT, NULL);
    }

    /* LCD + LVGL UI：必须最后初始化——lvgl_port_init 创建的 LVGL 任务会立即运行
     * timer 回调访问各模块快照，须先建好所有 mutex（C-13：失败降级串口自检） */
    if (lcd_driver_init() == ESP_OK) {
        ui_init();
    } else {
        ESP_LOGE(TAG, "LCD 初始化失败，仅串口自检");
    }

    xTaskCreate(sensor_task, "sensor_task", TASK_STACK_SENSOR, NULL, TASK_PRIO_SENSOR, NULL);
    xTaskCreate(diagnostics_task, "diag_task", TASK_STACK_DIAGNOSTIC, NULL, TASK_PRIO_DIAGNOSTIC, NULL);
}
