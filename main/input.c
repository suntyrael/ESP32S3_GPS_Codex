/*
 * input.c - 输入层实现：PCNT 编码器正交解码 + 按键事件状态机
 * 编码器每格 4 边沿 → count/4 为一格；左右旋分别触发 MODE_NEXT/MODE_PREV
 * 按键：10ms 轮询消抖；短按(<250ms)/中按(250~900ms)/长按(>900ms)/双击(间隔<400ms)
 */
#include "input.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "input";

static QueueHandle_t s_ev_queue = NULL;
static pcnt_unit_handle_t s_pcnt_unit = NULL;
static app_mode_t s_mode = MODE_BIKE_COMPUTER;

/* 按键状态机 */
#define KEY_DEBOUNCE_MS     20
#define KEY_SHORT_MAX_MS    250
#define KEY_MIDDLE_MAX_MS   900
#define KEY_DOUBLE_GAP_MS   400

esp_err_t input_init(void)
{
    s_ev_queue = xQueueCreate(16, sizeof(input_event_t));
    if (s_ev_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* PCNT 正交解码（ENC_A 边沿 + ENC_B 电平） */
    pcnt_unit_config_t unit_cfg = {
        .low_limit = -32768,
        .high_limit = 32767,
        .flags.accum_count = false,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_cfg, &s_pcnt_unit), TAG, "pcnt_new_unit failed");
    pcnt_chan_config_t chan_cfg = {
        .edge_gpio_num = PIN_ENC_A,
        .level_gpio_num = PIN_ENC_B,
    };
    pcnt_channel_handle_t chan = NULL;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(s_pcnt_unit, &chan_cfg, &chan), TAG, "pcnt_new_channel failed");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE),
                        TAG, "pcnt edge action failed");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
                        TAG, "pcnt level action failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(s_pcnt_unit), TAG, "pcnt_unit_enable failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(s_pcnt_unit), TAG, "pcnt_unit_start failed");

    /* 按键：输入 + 上拉（原理图已带上拉，内部再保底） */
    gpio_config_t key = {
        .pin_bit_mask = (1ULL << PIN_KEY_MAIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&key), TAG, "gpio_config(KEY) failed");

    xTaskCreate(input_task, "input_task", TASK_STACK_INPUT, NULL, TASK_PRIO_INPUT, NULL);
    ESP_LOGI(TAG, "input init ok (ENC 1/3, KEY 2)");
    return ESP_OK;
}

bool input_get_event(input_event_t *ev)
{
    return xQueueReceive(s_ev_queue, ev, 0) == pdTRUE;
}

app_mode_t input_get_mode(void)
{
    return s_mode;
}

static void post_event(input_event_t ev)
{
    xQueueSend(s_ev_queue, &ev, 0);
}

/* 输入任务：10ms 周期轮询编码器 + 按键状态机 */
static void input_task(void *arg)
{
    (void)arg;
    int prev_count = 0;
    pcnt_unit_clear_count(s_pcnt_unit);

    int key_pressed_ms = 0;
    bool key_was_pressed = false;
    int last_release_ms = -1000;    /* 上次释放时刻（双击检测） */

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10));

        /* ---- 编码器 ---- */
        int count = 0;
        if (pcnt_unit_get_count(s_pcnt_unit, &count) == ESP_OK && count != prev_count) {
            int delta = count - prev_count;
            prev_count = count;
            int steps = delta / 4;          /* 正交 4 边沿/格 */
            if (steps != 0) {
                if (steps > 0) {
                    post_event(INPUT_EV_MODE_NEXT);
                } else {
                    post_event(INPUT_EV_MODE_PREV);
                }
                s_mode = (app_mode_t)((s_mode + steps + MODE_MAX * 8) % MODE_MAX);
            }
        }

        /* ---- 按键状态机 ---- */
        bool pressed = (gpio_get_level(PIN_KEY_MAIN) == 0);
        if (pressed != key_was_pressed) {
            if (pressed) {
                key_pressed_ms = 0;         /* 按下开始 */
            } else {
                /* 释放：按持续时间分类 */
                if (key_pressed_ms < KEY_SHORT_MAX_MS) {
                    int now = (int)(esp_timer_get_time() / 1000);
                    if (now - last_release_ms < KEY_DOUBLE_GAP_MS) {
                        post_event(INPUT_EV_KEY_DOUBLE);
                    } else {
                        post_event(INPUT_EV_KEY_SHORT);
                    }
                    last_release_ms = now;
                } else if (key_pressed_ms < KEY_MIDDLE_MAX_MS) {
                    post_event(INPUT_EV_KEY_MIDDLE);
                } else {
                    post_event(INPUT_EV_KEY_LONG);
                }
            }
            key_was_pressed = pressed;
        } else if (pressed) {
            key_pressed_ms += 10;
        }
    }
}
