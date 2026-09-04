/*
 * input.c - 输入层实现：PCNT 编码器正交解码 + 按键事件状态机
 * 编码器每格 4 边沿 → count/4 为一格；左右旋分别触发 MODE_NEXT/MODE_PREV
 * 按键：10ms 轮询消抖；短按(<250ms)/中按(250~900ms)/长按(>900ms)/双击(间隔<400ms)
 */
#include "input.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "input";

static void input_task(void *arg);

static QueueHandle_t s_ev_queue = NULL;
static pcnt_unit_handle_t s_pcnt_unit = NULL;
static app_mode_t s_mode = MODE_MAIN;
static main_page_t s_main_page = MAIN_PAGE_PBOX;

/* 按键状态机参数统一收敛于 config.h (KEY_DEBOUNCE_MS / KEY_LONG_MIN_MS / KEY_DOUBLE_GAP_MS) */

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

    /* 机械编码器需上拉（PCNT 驱动默认不配置内部上拉；外部上拉缺失时电平不定 → 不触发） */
    gpio_config_t enc_pull = {
        .pin_bit_mask = (1ULL << PIN_ENC_A) | (1ULL << PIN_ENC_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&enc_pull);

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

void input_set_mode(app_mode_t m)
{
    if (m < MODE_MAX) {
        s_mode = m;
    }
}

main_page_t input_get_main_page(void)
{
    return s_main_page;
}

void input_set_main_page(main_page_t p)
{
    if (p < MAIN_PAGE_MAX) {
        s_main_page = p;
    }
}

static void post_event(input_event_t ev)
{
    xQueueSend(s_ev_queue, &ev, 0);
}

/* 输入任务：10ms 周期轮询编码器 + 按键状态机 */
#define ENC_PULSES_PER_STEP 4           /* 同向 4 脉冲 = 1 格 */
static void input_task(void *arg)
{
    (void)arg;
    int enc_last = 0;
    int enc_window = 0;                 /* 同向脉冲累计（符号=方向） */
    uint64_t enc_last_pulse_ms = 0;
    pcnt_unit_clear_count(s_pcnt_unit);

    int key_pressed_ms = 0;
    bool key_was_pressed = false;
    bool long_triggered = false;
    int last_release_ms = -1000;    /* 上次释放时刻（双击检测） */

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10));
        uint64_t now_ms = esp_timer_get_time() / 1000;

        /* ---- 编码器：同向 4 脉冲 = 1 格，判定后清零；方向反转重置 ---- */
        int count = 0;
        if (pcnt_unit_get_count(s_pcnt_unit, &count) == ESP_OK) {
            int delta = count - enc_last;
            enc_last = count;
            if (delta != 0) {
                enc_last_pulse_ms = now_ms;
                if ((enc_window > 0 && delta < 0) || (enc_window < 0 && delta > 0)) {
                    enc_window = delta;      /* 方向反转：重置为当前脉冲 */
                } else {
                    enc_window += delta;     /* 同向累计 */
                }
                if (enc_window >= ENC_PULSES_PER_STEP || enc_window <= -ENC_PULSES_PER_STEP) {
                    int dir = enc_window > 0 ? 1 : -1;
                    enc_window = 0;          /* 判定后清零计数器 */
                    post_event(dir > 0 ? INPUT_EV_MODE_NEXT : INPUT_EV_MODE_PREV);
                    ESP_LOGD(TAG, "enc pulse -> %s", dir > 0 ? "CW" : "CCW");
                }
            }
        }
        /* 窗口超时无脉冲 → 清零 */
        if (enc_window != 0 && now_ms - enc_last_pulse_ms > ENC_WINDOW_MS) {
            enc_window = 0;
        }

        /* ---- 按键状态机：满 800ms 立即触发长按，松手时静默复位 ---- */
        bool pressed = (gpio_get_level(PIN_KEY_MAIN) == 0);
        if (pressed != key_was_pressed) {
            if (pressed) {
                key_pressed_ms = 0;         /* 按下开始 */
                long_triggered = false;     /* 复位长按触发标记 */
            } else {
                /* 释放时刻：仅在未触发过长按的前提下处理短按与双击 */
                if (!long_triggered) {
                    int now = (int)(esp_timer_get_time() / 1000);
                    if (now - last_release_ms < KEY_DOUBLE_GAP_MS) {
                        post_event(INPUT_EV_KEY_DOUBLE);
                        ESP_LOGI(TAG, "key: DOUBLE");
                    } else {
                        post_event(INPUT_EV_KEY_SHORT);
                        ESP_LOGI(TAG, "key: SHORT (%d ms)", key_pressed_ms);
                    }
                    last_release_ms = now;
                } else {
                    ESP_LOGD(TAG, "key: LONG released after %d ms", key_pressed_ms);
                }
                long_triggered = false;
            }
            key_was_pressed = pressed;
        } else if (pressed) {
            key_pressed_ms += 10;
            /* 核心关键：一旦持续按下满 800ms，立即触发长按底层响应，绝不等松手！ */
            if (!long_triggered && key_pressed_ms >= KEY_LONG_MIN_MS) {
                long_triggered = true;
                post_event(INPUT_EV_KEY_LONG);
                ESP_LOGI(TAG, "key: LONG triggered immediately at %d ms", key_pressed_ms);
            }
        }
    }
}
