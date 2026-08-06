/*
 * app_main.c - 入口（阶段 0+1：构建基线 + 传感器自检）
 * 流程：NVS -> 传感器汇聚层 -> 创建 sensor_task / diagnostics_task
 */
#include "config.h"
#include "sensors.h"
#include "diagnostics.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "spi_flash_mmap.h"
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
    ESP_LOGI(TAG, "%s boot: %u cores, flash %uMB, IDF %s",
             FW_VERSION_STR, chip.cores, spi_flash_get_chip_size() / (1024 * 1024),
             esp_get_idf_version());

    ESP_ERROR_CHECK(nvs_init());

    esp_err_t ret = sensors_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "sensors_init 失败: %s", esp_err_to_name(ret));
    }

    xTaskCreate(sensor_task, "sensor_task", TASK_STACK_SENSOR, NULL, TASK_PRIO_SENSOR, NULL);
    xTaskCreate(diagnostics_task, "diag_task", TASK_STACK_DIAGNOSTIC, NULL, TASK_PRIO_DIAGNOSTIC, NULL);
}
