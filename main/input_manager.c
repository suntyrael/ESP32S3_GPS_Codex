#include "input_manager.h"

#include <stdlib.h>

#include "driver/gpio.h"
#include "esp_bit_defs.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "config.h"
#include "diagnostics.h"

static const char *TAG = "input";
static QueueHandle_t s_input_queue;
static portMUX_TYPE s_encoder_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile int32_t s_encoder_accum;
static volatile TickType_t s_last_encoder_tick;

static void enqueue_event(input_event_type_t type, int32_t value, uint32_t duration_ms) {
    if (!s_input_queue) {
        return;
    }
    input_event_t event = {
        .type = type,
        .value = value,
        .duration_ms = duration_ms,
    };
    if (xQueueSend(s_input_queue, &event, 0) == pdTRUE) {
        diagnostics_trigger_event((type <= INPUT_EVENT_ENCODER_RIGHT) ? "encoder" : "button", duration_ms);
    }
}

static void IRAM_ATTR encoder_isr(void *arg) {
    (void)arg;
    int level_a = gpio_get_level(CONFIG_ENCODER_GPIO_A);
    int level_b = gpio_get_level(CONFIG_ENCODER_GPIO_B);
    int direction = (level_a == level_b) ? 1 : -1;
    portENTER_CRITICAL_ISR(&s_encoder_lock);
    s_encoder_accum += direction;
    s_last_encoder_tick = xTaskGetTickCountFromISR();
    portEXIT_CRITICAL_ISR(&s_encoder_lock);
}

static void process_encoder(void) {
    int32_t delta = 0;
    TickType_t last_tick = 0;
    int32_t steps = 0;
    portENTER_CRITICAL(&s_encoder_lock);
    delta = s_encoder_accum;
    last_tick = s_last_encoder_tick;
    if (abs(delta) >= CONFIG_ENCODER_STEP_WINDOW) {
        steps = delta / CONFIG_ENCODER_STEP_WINDOW;
        s_encoder_accum -= steps * CONFIG_ENCODER_STEP_WINDOW;
    }
    portEXIT_CRITICAL(&s_encoder_lock);

    if (steps == 0) {
        TickType_t now = xTaskGetTickCount();
        if ((now - last_tick) > pdMS_TO_TICKS(CONFIG_ENCODER_IDLE_CLEAR_MS)) {
            portENTER_CRITICAL(&s_encoder_lock);
            s_encoder_accum = 0;
            portEXIT_CRITICAL(&s_encoder_lock);
        }
        return;
    }
    if (steps > 0) {
        for (int i = 0; i < steps; ++i) {
            enqueue_event(INPUT_EVENT_ENCODER_RIGHT, 1, 0);
        }
    } else if (steps < 0) {
        for (int i = 0; i > steps; --i) {
            enqueue_event(INPUT_EVENT_ENCODER_LEFT, -1, 0);
        }
    }
}

static void encoder_task(void *arg) {
    (void)arg;
    while (1) {
        process_encoder();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void button_task(void *arg) {
    (void)arg;
    bool last_state = true;
    TickType_t last_change = 0;
    TickType_t press_tick = 0;
    bool pending_double = false;
    TickType_t pending_tick = 0;
    uint32_t pending_duration = 0;
    while (1) {
        bool level = gpio_get_level(CONFIG_BUTTON_GPIO);
        TickType_t now = xTaskGetTickCount();
        if (level != last_state) {
            if ((now - last_change) >= pdMS_TO_TICKS(CONFIG_BUTTON_DEBOUNCE_MS)) {
                last_state = level;
                last_change = now;
                if (!level) {
                    press_tick = now;
                } else {
                    uint32_t duration_ms = (uint32_t)(((now - press_tick) * 1000) / configTICK_RATE_HZ);
                    pending_duration = duration_ms;
                    if (duration_ms >= CONFIG_BUTTON_LONG_MS) {
                        enqueue_event(INPUT_EVENT_BUTTON_LONG, 0, duration_ms);
                        pending_double = false;
                    } else if (duration_ms >= CONFIG_BUTTON_MEDIUM_MS) {
                        enqueue_event(INPUT_EVENT_BUTTON_MEDIUM, 0, duration_ms);
                        pending_double = false;
                    } else {
                        if (pending_double && (now - pending_tick) <= pdMS_TO_TICKS(CONFIG_BUTTON_DOUBLE_GAP_MS)) {
                            enqueue_event(INPUT_EVENT_BUTTON_DOUBLE, 0, duration_ms);
                            pending_double = false;
                        } else {
                            pending_double = true;
                            pending_tick = now;
                        }
                    }
                }
            }
        }

        if (pending_double && (now - pending_tick) > pdMS_TO_TICKS(CONFIG_BUTTON_DOUBLE_GAP_MS)) {
            enqueue_event(INPUT_EVENT_BUTTON_SHORT, 0, pending_duration);
            pending_double = false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void init_gpio(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = BIT64(CONFIG_BUTTON_GPIO) | BIT64(CONFIG_ENCODER_GPIO_A) | BIT64(CONFIG_ENCODER_GPIO_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_intr_type(CONFIG_ENCODER_GPIO_A, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(CONFIG_ENCODER_GPIO_B, GPIO_INTR_DISABLE);
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install ISR service (%d)", err);
    }
    gpio_isr_handler_add(CONFIG_ENCODER_GPIO_A, encoder_isr, NULL);
}

void input_manager_init(void) {
    init_gpio();
    s_input_queue = xQueueCreate(16, sizeof(input_event_t));
    xTaskCreatePinnedToCore(encoder_task, "enc", CONFIG_TASK_STACK_DEFAULT, NULL, CONFIG_TASK_PRIO_INPUT, NULL, 0);
    xTaskCreatePinnedToCore(button_task, "button", CONFIG_TASK_STACK_DEFAULT, NULL, CONFIG_TASK_PRIO_INPUT, NULL, 0);
    ESP_LOGI(TAG, "Input manager initialized");
}

bool input_manager_get_event(input_event_t *event_out, TickType_t ticks_to_wait) {
    if (!s_input_queue || !event_out) {
        return false;
    }
    return xQueueReceive(s_input_queue, event_out, ticks_to_wait) == pdTRUE;
}

