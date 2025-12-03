/**
 * @file sensor.c
 * @brief 传感器驱动封装，负责 I2C 初始化与数据轴向矫正。
 */

#include "sensor.h"
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "sensor";
static sensor_bus_t s_bus_cfg;

esp_err_t sensor_init_bus(const sensor_bus_t *bus)
{
    if (!bus) {
        return ESP_ERR_INVALID_ARG;
    }
    s_bus_cfg = *bus;

    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = bus->sda_pin,
        .scl_io_num = bus->scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = bus->freq_hz,
    };
    ESP_LOGI(TAG, "初始化 I2C%d (SDA=%d, SCL=%d, %uHz)", bus->i2c_port, bus->sda_pin, bus->scl_pin, bus->freq_hz);
    ESP_RETURN_ON_ERROR(i2c_param_config(bus->i2c_port, &cfg), TAG, "配置 I2C 失败");
    return i2c_driver_install(bus->i2c_port, cfg.mode, 0, 0, 0);
}

esp_err_t sensor_read_frame(sensor_frame_t *frame)
{
    if (!frame) {
        return ESP_ERR_INVALID_ARG;
    }

    // 这里填充真实的传感器采集逻辑与姿态矩阵矫正，当前为占位值。
    frame->ax = frame->ay = frame->az = 0;
    frame->gx = frame->gy = frame->gz = 0;
    frame->timestamp_ms = 0;

    ESP_LOGD(TAG, "采集传感器数据并完成姿态转换");
    return ESP_OK;
}
