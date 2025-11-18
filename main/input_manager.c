#include "input_manager.h"

// 这是根据远程仓库 suntyrael/ESP32S3_GPS_Codex 的 input_manager.c 改写的版本，
// 增加了旋转编码器的方向和计数支持，并保留按键四种状态检测。

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
// 用于保护编码器累积计数的自旋锁
static portMUX_TYPE s_encoder_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile int32_t s_encoder_accum;
static volatile TickType_t s_last_encoder_tick;

/**
 * @brief 将输入事件发送到队列，同时触发诊断事件。
 *
 * @param type   事件类型
 * @param value  计数值（旋转编码器步数）；按键事件可填 0
 * @param duration_ms 按下时长（按键事件），旋转编码器事件可填 0
 */
static void enqueue_event(input_event_type_t type, int32_t value, uint32_t duration_ms) {
    if (!s_input_queue) {
        return;
    }
    input_event_t event = {
        .type        = type,
        .value       = value,
        .duration_ms = duration_ms,
    };
    if (xQueueSend(s_input_queue, &event, 0) == pdTRUE) {
        diagnostics_trigger_event((type <= INPUT_EVENT_ENCODER_RIGHT) ? "encoder" : "button", duration_ms);
    }
}

/**
 * @brief 旋转编码器中断服务程序
 *
 * 通过读取两相电平判断旋转方向，并累加计数。此函数在中断上下文中执行。
 */
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

/**
 * @brief 处理旋转编码器累积计数并生成事件
 *
 * 如果累积计数达到窗口阈值，则计算步数并清零相应计数。随后一次性发送一个事件，并
 * 将步数作为 value。这样既包含方向，也携带计数值，避免发送多个事件。
 */
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
    // 发送一次事件，value 为步数，方向由正负表示
    if (steps > 0) {
        enqueue_event(INPUT_EVENT_ENCODER_RIGHT, steps, 0);
    } else {
        enqueue_event(INPUT_EVENT_ENCODER_LEFT, steps, 0);
    }
}

/**
 * @brief 编码器处理任务
 */
static void encoder_task(void *arg) {
    (void)arg;
    while (1) {
        process_encoder();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/**
 * @brief 按键处理任务，检测短按、中按、长按和双击
 */
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
        // 如果等待双击超时，则认定为单击
        if (pending_double && (now - pending_tick) > pdMS_TO_TICKS(CONFIG_BUTTON_DOUBLE_GAP_MS)) {
            enqueue_event(INPUT_EVENT_BUTTON_SHORT, 0, pending_duration);
            pending_double = false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief 初始化 GPIO
 */
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

/**
 * @brief 输入管理器初始化
 */
void input_manager_init(void) {
    init_gpio();
    s_input_queue = xQueueCreate(16, sizeof(input_event_t));
    xTaskCreatePinnedToCore(encoder_task, "enc", CONFIG_TASK_STACK_DEFAULT, NULL, CONFIG_TASK_PRIO_INPUT, NULL, 0);
    xTaskCreatePinnedToCore(button_task, "button", CONFIG_TASK_STACK_DEFAULT, NULL, CONFIG_TASK_PRIO_INPUT, NULL, 0);
    ESP_LOGI(TAG, "Input manager initialized");
}

/**
 * @brief 获取输入事件
 */
bool input_manager_get_event(input_event_t *event_out, TickType_t ticks_to_wait) {
    if (!s_input_queue || !event_out) {
        return false;
    }
    return xQueueReceive(s_input_queue, event_out, ticks_to_wait) == pdTRUE;
}