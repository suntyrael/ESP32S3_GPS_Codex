#include "i2c_bus.h"
#include "config.h"
#include "esp_log.h"

static const char *TAG = "i2c_bus";

esp_err_t i2c_bus_init(i2c_master_bus_handle_t *out_handle)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = I2C_BUS_GLITCH_CNT,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, out_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "I2C bus ready: SCL=%d SDA=%d @%d Hz", PIN_I2C_SCL, PIN_I2C_SDA, I2C_BUS_CLK_HZ);
    return ESP_OK;
}
