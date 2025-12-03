/**
 * @file main.c
 * @brief 应用入口，初始化硬件、创建 FreeRTOS 任务与主状态机框架。
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/sdmmc_host.h"
#include "driver/uart.h"

#include "gnss.h"
#include "sensor.h"
#include "input.h"
#include "ui.h"
#include "gpx.h"
#include "pgear.h"
#include "rtc.h"
#include "log.h"
#include "power.h"

static const char *TAG = "main";

// SRS 硬件表占位映射，后续可根据实际原理图调整。
#define PIN_GNSS_TX     GPIO_NUM_17
#define PIN_GNSS_RX     GPIO_NUM_18
#define PIN_GNSS_PPS    GPIO_NUM_16
#define PIN_I2C_SDA     GPIO_NUM_8
#define PIN_I2C_SCL     GPIO_NUM_9
#define PIN_SPI_MOSI    GPIO_NUM_11
#define PIN_SPI_MISO    GPIO_NUM_13
#define PIN_SPI_SCLK    GPIO_NUM_12
#define PIN_SDIO_PWR    GPIO_NUM_2
#define PIN_SENSOR_PWR  GPIO_NUM_3
#define PIN_BACKLIGHT   GPIO_NUM_4
#define PIN_BUTTON      GPIO_NUM_5
#define PIN_ENCODER_A   GPIO_NUM_6
#define PIN_ENCODER_B   GPIO_NUM_7

typedef enum {
    APP_STATE_BOOT = 0,
    APP_STATE_IDLE,
    APP_STATE_TRACKING,
    APP_STATE_GPX,
    APP_STATE_PGEAR,
    APP_STATE_ERROR,
} app_state_t;

static QueueHandle_t s_gnss_queue;
static QueueHandle_t s_sensor_queue;
static QueueHandle_t s_input_queue;
static SemaphoreHandle_t s_state_lock;
static app_state_t s_state = APP_STATE_BOOT;

static void app_state_machine(void)
{
    switch (s_state) {
    case APP_STATE_BOOT:
        ESP_LOGI(TAG, "状态机: BOOT -> IDLE");
        s_state = APP_STATE_IDLE;
        break;
    case APP_STATE_IDLE:
        // 根据输入事件或任务数据转入其它状态
        break;
    case APP_STATE_TRACKING:
    case APP_STATE_GPX:
    case APP_STATE_PGEAR:
        // 预留状态处理逻辑
        break;
    case APP_STATE_ERROR:
    default:
        ESP_LOGW(TAG, "进入异常状态，等待复位");
        break;
    }
}

static void gnss_task(void *arg)
{
    gnss_fix_t fix;
    while (1) {
        if (gnss_poll_data(&fix) == ESP_OK) {
            xQueueSend(s_gnss_queue, &fix, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void sensor_task(void *arg)
{
    sensor_frame_t frame;
    while (1) {
        if (sensor_read_frame(&frame) == ESP_OK) {
            xQueueSend(s_sensor_queue, &frame, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void input_task(void *arg)
{
    input_event_t evt;
    while (1) {
        if (input_poll_event(&evt) == ESP_OK && evt.type != INPUT_EVENT_NONE) {
            xQueueSend(s_input_queue, &evt, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void rtc_task(void *arg)
{
    gnss_fix_t fix;
    while (1) {
        if (xQueueReceive(s_gnss_queue, &fix, pdMS_TO_TICKS(100)) == pdTRUE && fix.fix_valid) {
            rtc_sync_from_gnss(fix.timestamp_ms);
        }
    }
}

static void ui_task(void *arg)
{
    gnss_fix_t fix;
    input_event_t evt;
    while (1) {
        if (xQueueReceive(s_input_queue, &evt, pdMS_TO_TICKS(10)) == pdTRUE) {
            ui_handle_input(evt.type);
        }
        if (xQueuePeek(s_gnss_queue, &fix, 0) == pdTRUE) {
            ui_update(fix.fix_valid);
        } else {
            ui_update(false);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void heartbeat_task(void *arg)
{
    while (1) {
        log_heartbeat();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void hardware_init(void)
{
    // GPIO 预配置
    gpio_config_t spi_pins = {
        .pin_bit_mask = (1ULL << PIN_SPI_MOSI) | (1ULL << PIN_SPI_MISO) | (1ULL << PIN_SPI_SCLK),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&spi_pins);

    // SDIO/SDMMC 占位初始化
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    (void)host; // 避免未使用警告，具体实现取决于实际硬件。

    power_config_t power_cfg = {
        .sdio_power_pin = PIN_SDIO_PWR,
        .sensor_power_pin = PIN_SENSOR_PWR,
        .backlight_pwm_pin = PIN_BACKLIGHT,
    };
    power_init(&power_cfg);

    sensor_bus_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_pin = PIN_I2C_SDA,
        .scl_pin = PIN_I2C_SCL,
        .freq_hz = 400000,
    };
    sensor_init_bus(&bus_cfg);

    input_config_t input_cfg = {
        .button_pin = PIN_BUTTON,
        .encoder_a_pin = PIN_ENCODER_A,
        .encoder_b_pin = PIN_ENCODER_B,
        .debounce_ms = 30,
    };
    input_init(&input_cfg);

    rtc_config_t rtc_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_pin = PIN_I2C_SDA,
        .scl_pin = PIN_I2C_SCL,
    };
    rtc_init(&rtc_cfg);

    gnss_config_t gnss_cfg = {
        .uart_port = UART_NUM_1,
        .tx_pin = PIN_GNSS_TX,
        .rx_pin = PIN_GNSS_RX,
        .baudrate = 115200,
        .pps_pin = PIN_GNSS_PPS,
        .enable_ubx = true,
    };
    gnss_init(&gnss_cfg);
    gnss_configure_baudrate(gnss_cfg.baudrate);

    ui_init();
    pgear_init();
    gpx_begin();
}

void app_main(void)
{
    ESP_LOGI(TAG, "应用启动，建立任务与资源");
    hardware_init();

    s_gnss_queue = xQueueCreate(4, sizeof(gnss_fix_t));
    s_sensor_queue = xQueueCreate(4, sizeof(sensor_frame_t));
    s_input_queue = xQueueCreate(4, sizeof(input_event_t));
    s_state_lock = xSemaphoreCreateMutex();

    xTaskCreate(gnss_task, "gnss_task", 4096, NULL, 5, NULL);
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(input_task, "input_task", 2048, NULL, 6, NULL);
    xTaskCreate(rtc_task, "rtc_task", 2048, NULL, 4, NULL);
    xTaskCreate(ui_task, "ui_task", 4096, NULL, 4, NULL);
    xTaskCreate(heartbeat_task, "heartbeat_task", 2048, NULL, 3, NULL);

    while (1) {
        if (xSemaphoreTake(s_state_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
            app_state_machine();
            xSemaphoreGive(s_state_lock);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
