#include "lis2mdl.h"
#include "config.h"
#include "esp_log.h"
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "lis2mdl";

/* 寄存器（LIS2MDL） */
#define REG_WHO_AM_I        0x4F
#define REG_CFG_REG_A       0x60
#define REG_CFG_REG_B       0x61
#define REG_STATUS_REG      0x67
#define REG_OUTX_L_REG      0x68
#define REG_TEMP_OUT_L      0x6E

struct lis2mdl_dev {
    i2c_master_dev_handle_t i2c_dev;
};

static esp_err_t write_reg(lis2mdl_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev->i2c_dev, buf, sizeof(buf), I2C_XFER_TIMEOUT_MS);
}

static esp_err_t read_regs(lis2mdl_handle_t dev, uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, out, len, I2C_XFER_TIMEOUT_MS);
}

esp_err_t lis2mdl_init(i2c_master_bus_handle_t bus, lis2mdl_handle_t *out)
{
    if (bus == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = NULL;

    if (i2c_master_probe(bus, MAG_I2C_ADDR, I2C_PROBE_TIMEOUT_MS) != ESP_OK) {
        ESP_LOGE(TAG, "0x%02X 无 ACK", MAG_I2C_ADDR);
        return ESP_ERR_NOT_FOUND;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MAG_I2C_ADDR,
        .scl_speed_hz = I2C_BUS_CLK_HZ,
    };
    lis2mdl_handle_t dev = calloc(1, sizeof(*dev));
    if (dev == NULL) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &dev->i2c_dev);
    if (ret != ESP_OK) {
        free(dev);
        return ret;
    }
    uint8_t who = 0;
    ret = read_regs(dev, REG_WHO_AM_I, &who, 1);
    if (ret != ESP_OK || who != MAG_WHO_AM_I) {
        ESP_LOGE(TAG, "WHO_AM_I=0x%02X（期望 0x%02X）", who, MAG_WHO_AM_I);
        i2c_master_bus_rm_device(dev->i2c_dev);
        free(dev);
        return ESP_ERR_INVALID_RESPONSE;
    }
    /* CFG_REG_B: IF_ADD_INC=1（多字节读自增） */
    write_reg(dev, REG_CFG_REG_B, 0x02);
    /* CFG_REG_A: 连续模式 10Hz（MD=00, ODR=00） */
    write_reg(dev, REG_CFG_REG_A, 0x00);

    ESP_LOGI(TAG, "LIS2MDL ready @0x%02X, WHO_AM_I=0x%02X", MAG_I2C_ADDR, who);
    *out = dev;
    return ESP_OK;
}

esp_err_t lis2mdl_read(lis2mdl_handle_t dev, lis2mdl_data_t *data)
{
    if (dev == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 连续失败计数：用于触发重新初始化（自恢复） */
    static uint8_t s_consec_fail = 0;

    for (int attempt = 0; attempt < 3; attempt++) {
        uint8_t status = 0;
        /* 等数据就绪（ZYXDA，bit0），超时 100ms */
        for (int i = 0; i < 20; i++) {
            if (read_regs(dev, REG_STATUS_REG, &status, 1) != ESP_OK) {
                status = 0;
                break;
            }
            if (status & 0x01) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        if (!(status & 0x01)) {
            continue;   /* 未就绪/读失败 → 重试 */
        }

        uint8_t buf[6];
        if (read_regs(dev, REG_OUTX_L_REG, buf, sizeof(buf)) != ESP_OK) {
            continue;
        }
        for (int i = 0; i < 3; i++) {
            int16_t v = (int16_t)((uint16_t)buf[2 * i] | ((uint16_t)buf[2 * i + 1] << 8));
            data->mag_mgauss[i] = (float)v * 1.5f;     /* 1.5 mGauss/LSB */
        }

        if (read_regs(dev, REG_TEMP_OUT_L, buf, 2) != ESP_OK) {
            continue;   /* 温度读失败 → 重试整帧 */
        }
        int16_t t = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
        data->temp_c = 25.0f + (float)t / 8.0f;         /* 8 LSB/℃，0=25℃ */

        s_consec_fail = 0;
        return ESP_OK;
    }

    s_consec_fail++;
    if (s_consec_fail >= 3) {
        /* 连续失败：总线恢复（9 SCL 脉冲）+ 重新进入连续模式自恢复 */
        ESP_LOGW(TAG, "连续读取失败，执行总线恢复+重初始化");
        i2c_master_bus_handle_t bus = NULL;
        if (i2c_master_get_bus_handle(0, &bus) == ESP_OK && bus != NULL) {
            i2c_master_bus_reset(bus);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        write_reg(dev, REG_CFG_REG_B, 0x02);   /* IF_ADD_INC */
        write_reg(dev, REG_CFG_REG_A, 0x00);   /* 连续模式 10Hz */
        s_consec_fail = 0;
    }
    return ESP_ERR_TIMEOUT;
}
