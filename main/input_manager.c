#include "input_manager.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "config.h"
#include "diagnostics.h"

static const char *TAG = "input";
static QueueHandle_t s_input_queue;

static void enqueue_event(input_event_type_t type, int32_t value, uint32_t duration_ms) {
    if (!s_input_queue) {
        return;
    }
    input_event_t event = {
        .type = type,
        .value = value,
        .duration_ms = duration_ms,
    };
    xQueueSend(s_input_queue, &event, 0);
    diagnostics_trigger_event("input", duration_ms);
}

static void input_task(void *arg) {
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(200));
        enqueue_event(INPUT_EVENT_BUTTON_SHORT, 0, CONFIG_BUTTON_SHORT_MS);
    }
}

void input_manager_init(void) {
    s_input_queue = xQueueCreate(8, sizeof(input_event_t));
    xTaskCreatePinnedToCore(input_task, "input", CONFIG_TASK_STACK_DEFAULT, NULL, CONFIG_TASK_PRIO_INPUT, NULL, 0);
    ESP_LOGI(TAG, "Input manager initialized");
}

bool input_manager_get_event(input_event_t *event_out, TickType_t ticks_to_wait) {
    if (!s_input_queue || !event_out) {
        return false;
    }
    return xQueueReceive(s_input_queue, event_out, ticks_to_wait) == pdTRUE;
}

