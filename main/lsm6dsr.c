#include "lsm6dsr.h"
#include "config.h"
#include "esp_log.h"
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* 寄存器值已与 ST 官方驱动（STMicroelectronics/lsm6dsrx-pid）交叉验证：
 * WHO_AM_I=0x6B（LSM6DSRX_ID）、ODR_XL/ODR_G 104Hz=0100、FS_XL ±2g=00、
 * fs_g 为 4 位字段（±2000dps=0xC，注意非 3 位） */
static const char *TAG = "lsm6dsr";

/* 寄存器（LSM6DSR） */
#define REG_WHO_AM_I        0x0F
#define REG_CTRL1_XL        0x10
#define REG_CTRL2_G         0x11
#define REG_CTRL3_C         0x12
#define REG_OUT_TEMP_L      0x20
#define REG_OUTX_L_G        0x22
#define REG_OUTX_L_XL       0x28

struct lsm6dsr_dev {
    i2c_master_dev_handle_t i2c_dev;
};

static esp_err_t write_reg(lsm6dsr_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev->i2c_dev, buf, sizeof(buf), I2C_XFER_TIMEOUT_MS);
}

static esp_err_t read_regs(lsm6dsr_handle_t dev, uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, out, len, I2C_XFER_TIMEOUT_MS);
}

/* 轴向重映射（config.h 符号可配） */
static void remap_axes(float in[3], float out[3])
{
    out[0] = in[0] * IMU_AXIS_X_SIGN;
    out[1] = in[1] * IMU_AXIS_Y_SIGN;
    out[2] = in[2] * IMU_AXIS_Z_SIGN;
}

esp_err_t lsm6dsr_init(i2c_master_bus_handle_t bus, lsm6dsr_handle_t *out)
{
    if (bus == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = NULL;

    const uint8_t addrs[2] = { IMU_I2C_ADDR_A, IMU_I2C_ADDR_B };
    for (size_t i = 0; i < 2; i++) {
        /* 地址探测：ACK 即可能命中 */
        if (i2c_master_probe(bus, addrs[i], I2C_PROBE_TIMEOUT_MS) != ESP_OK) {
            continue;
        }
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addrs[i],
            .scl_speed_hz = I2C_BUS_CLK_HZ,
        };
        lsm6dsr_handle_t dev = calloc(1, sizeof(*dev));
        if (dev == NULL) {
            return ESP_ERR_NO_MEM;
        }
        esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &dev->i2c_dev);
        if (ret != ESP_OK) {
            free(dev);
            continue;
        }
        /* WHO_AM_I 宽松校验 */
        uint8_t who = 0;
        if (read_regs(dev, REG_WHO_AM_I, &who, 1) != ESP_OK) {
            free(dev);
            continue;
        }
        if (who != IMU_WHO_AM_I_1 && who != IMU_WHO_AM_I_2) {
            ESP_LOGW(TAG, "addr 0x%02X: WHO_AM_I=0x%02X 未知", addrs[i], who);
            i2c_master_bus_rm_device(dev->i2c_dev);
            free(dev);
            continue;
        }
        /* 软复位 + 等 10ms */
        write_reg(dev, REG_CTRL3_C, 0x01);
        vTaskDelay(pdMS_TO_TICKS(10));
        /* BDU=1 + IF_INC=1 */
        write_reg(dev, REG_CTRL3_C, 0x44);
        /* 加速度：104Hz（ODR_XL=0100），±2g（FS_XL=00）→ 0x40 */
        write_reg(dev, REG_CTRL1_XL, 0x40);
        /* 陀螺：104Hz（ODR_G=0100），±2000dps（fs_g 4 位字段=0xC）→ 0x4C */
        write_reg(dev, REG_CTRL2_G, 0x4C);

        ESP_LOGI(TAG, "LSM6DSR ready @0x%02X, WHO_AM_I=0x%02X", addrs[i], who);
        *out = dev;
        return ESP_OK;
    }
    ESP_LOGE(TAG, "LSM6DSR 未找到（探测 0x6A/0x6B 失败或 ID 不匹配）");
    return ESP_ERR_NOT_FOUND;
}

esp_err_t lsm6dsr_read(lsm6dsr_handle_t dev, lsm6dsr_data_t *data)
{
    if (dev == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t buf[6];
    esp_err_t ret;

    /* 加速度：OUTX_L_XL 起 6 字节（小端 int16） */
    ret = read_regs(dev, REG_OUTX_L_XL, buf, sizeof(buf));
    if (ret != ESP_OK) {
        return ret;
    }
    float raw_mg[3];
    for (int i = 0; i < 3; i++) {
        int16_t v = (int16_t)((uint16_t)buf[2 * i] | ((uint16_t)buf[2 * i + 1] << 8));
        raw_mg[i] = (float)v * 0.061f;      /* ±2g: 0.061 mg/LSB */
    }
    remap_axes(raw_mg, data->accel_mg);

    /* 陀螺：OUTX_L_G 起 6 字节 */
    ret = read_regs(dev, REG_OUTX_L_G, buf, sizeof(buf));
    if (ret != ESP_OK) {
        return ret;
    }
    float raw_mdps[3];
    for (int i = 0; i < 3; i++) {
        int16_t v = (int16_t)((uint16_t)buf[2 * i] | ((uint16_t)buf[2 * i + 1] << 8));
        raw_mdps[i] = (float)v * 70.0f;     /* ±2000dps: 70 mdps/LSB */
    }
    remap_axes(raw_mdps, data->gyro_mdps);

    /* 温度：OUT_TEMP_L 2 字节，0x0000=25℃，1 LSB=1/256 ℃ */
    ret = read_regs(dev, REG_OUT_TEMP_L, buf, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    int16_t t = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    data->temp_c = 25.0f + (float)t / 256.0f;

    return ESP_OK;
}
