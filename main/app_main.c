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
                pbox_arm();     /* P-Box 模式：READY↔ARMED/FINISHED */
                break;
            default:
                break;          /* 其余事件由 UI 轮询模式变化处理 */
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
    ESP_LOGI(TAG, "%s boot: %u cores, IDF %s",
             FW_VERSION_STR, (unsigned)chip.cores, esp_get_idf_version());

    ESP_ERROR_CHECK(nvs_init());

    esp_err_t ret = sensors_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "sensors_init 失败: %s", esp_err_to_name(ret));
    }

    /* LCD + LVGL UI（阶段 2）；失败不阻塞传感器自检（C-13 降级） */
    if (lcd_driver_init() == ESP_OK) {
        ui_init();
    } else {
        ESP_LOGE(TAG, "LCD 初始化失败，仅串口自检");
    }

    /* GNSS（阶段 3）：LDO 使能 + UART1 + 解析任务；失败降级 */
    if (gnss_init() != ESP_OK) {
        ESP_LOGE(TAG, "GNSS 初始化失败，降级 N/A");
    }

    /* 输入 + P-Box（阶段 4） */
    if (input_init() == ESP_OK) {
        xTaskCreate(app_task, "app_task", TASK_STACK_INPUT, NULL, TASK_PRIO_INPUT, NULL);
    }
    pbox_init();

    xTaskCreate(sensor_task, "sensor_task", TASK_STACK_SENSOR, NULL, TASK_PRIO_SENSOR, NULL);
    xTaskCreate(diagnostics_task, "diag_task", TASK_STACK_DIAGNOSTIC, NULL, TASK_PRIO_DIAGNOSTIC, NULL);
}
