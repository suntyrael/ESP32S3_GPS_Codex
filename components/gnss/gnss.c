/**
 * @file gnss.c
 * @brief GNSS 驱动实现，封装 UART1 收发、NMEA/UBX 解析以及 PPS 处理。
 */

#include "gnss.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

static const char *TAG = "gnss";
static gnss_config_t s_gnss_config;

esp_err_t gnss_init(const gnss_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    s_gnss_config = *config;

    uart_config_t uart_config = {
        .baud_rate = config->baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_LOGI(TAG, "初始化 UART%d (TX=%d, RX=%d, baud=%d)", config->uart_port, config->tx_pin, config->rx_pin, config->baudrate);
    ESP_RETURN_ON_ERROR(uart_driver_install(config->uart_port, 4096, 0, 0, NULL, 0), TAG, "无法安装 UART 驱动");
    ESP_RETURN_ON_ERROR(uart_param_config(config->uart_port, &uart_config), TAG, "无法配置 UART 参数");
    ESP_RETURN_ON_ERROR(uart_set_pin(config->uart_port, config->tx_pin, config->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "无法设置引脚");

    if (config->pps_pin >= 0) {
        gpio_config_t io_conf = {
            .intr_type = GPIO_INTR_POSEDGE,
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = 1ULL << config->pps_pin,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_ENABLE,
        };
        gpio_config(&io_conf);
    }

    ESP_LOGI(TAG, "GNSS 模块初始化完成，等待卫星锁定");
    return ESP_OK;
}

esp_err_t gnss_poll_data(gnss_fix_t *out_fix)
{
    if (!out_fix) {
        return ESP_ERR_INVALID_ARG;
    }

    // 在实际实现中解析 NMEA 和 UBX 数据帧，这里仅给出占位示例。
    out_fix->latitude = 0;
    out_fix->longitude = 0;
    out_fix->altitude = 0;
    out_fix->speed_kmh = 0;
    out_fix->heading_deg = 0;
    out_fix->timestamp_ms = 0;
    out_fix->fix_valid = false;

    int length = 0;
    uint8_t buf[256];
    length = uart_read_bytes(s_gnss_config.uart_port, buf, sizeof(buf), pdMS_TO_TICKS(20));
    if (length > 0) {
        ESP_LOGD(TAG, "接收到 %d 字节 GNSS 数据用于解析", length);
    }

    return ESP_OK;
}

esp_err_t gnss_configure_baudrate(int baudrate)
{
    ESP_LOGI(TAG, "切换 GNSS 波特率到 %d", baudrate);
    s_gnss_config.baudrate = baudrate;
    return uart_set_baudrate(s_gnss_config.uart_port, baudrate);
}
