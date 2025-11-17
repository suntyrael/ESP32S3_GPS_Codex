#include "gpx_logger.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "config.h"

typedef struct {
    sensors_state_t snapshot;
} gpx_sample_t;

static const char *TAG = "gpx";
static gpx_logger_state_t s_state = GPX_LOGGER_STATE_IDLE;
static QueueHandle_t s_sample_queue;

static void gpx_task(void *arg) {
    (void)arg;
    gpx_sample_t sample;
    while (1) {
        if (xQueueReceive(s_sample_queue, &sample, portMAX_DELAY) == pdTRUE) {
            if (s_state == GPX_LOGGER_STATE_RECORDING) {
                ESP_LOGI(TAG, "GPX sample lat=%.5f lon=%.5f speed=%.1f", sample.snapshot.gnss.latitude_deg,
                         sample.snapshot.gnss.longitude_deg, sample.snapshot.gnss.speed_kmh);
                // TODO: 实际写入 SD 卡上的 GPX 文件。
            }
        }
    }
}

void gpx_logger_init(void) {
    s_sample_queue = xQueueCreate(16, sizeof(gpx_sample_t));
    xTaskCreatePinnedToCore(gpx_task, "gpx", CONFIG_TASK_STACK_DEFAULT, NULL, CONFIG_TASK_PRIO_LOGGER, NULL, 0);
    ESP_LOGI(TAG, "GPX logger ready at %s", CONFIG_GPX_DIRECTORY);
}

void gpx_logger_start(void) {
    if (s_state != GPX_LOGGER_STATE_RECORDING) {
        s_state = GPX_LOGGER_STATE_RECORDING;
        ESP_LOGI(TAG, "GPX recording started");
    }
}

void gpx_logger_stop(void) {
    if (s_state == GPX_LOGGER_STATE_RECORDING) {
        s_state = GPX_LOGGER_STATE_IDLE;
        ESP_LOGI(TAG, "GPX recording stopped");
    }
}

void gpx_logger_push_sample(const sensors_state_t *state) {
    if (!state || !s_sample_queue) {
        return;
    }
    gpx_sample_t sample = {.snapshot = *state};
    xQueueSend(s_sample_queue, &sample, 0);
}

gpx_logger_state_t gpx_logger_get_state(void) {
    return s_state;
}

