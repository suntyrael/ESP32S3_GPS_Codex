#include "bmp388.h"
#include "config.h"
#include "esp_log.h"
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "bmp388";

/* 寄存器（BMP388，地址对照 Bosch bmp3_defs.h） */
#define REG_CHIP_ID         0x00
#define REG_STATUS          0x03
#define REG_PRESS_XLSB      0x04        /* 起读 6 字节：P XLSB/LSB/MSB, T XLSB/LSB/MSB */
#define REG_PWR_CTRL        0x1B        /* bit0=press_en, bit1=temp_en, bit[5:4]=mode */
#define REG_OSR             0x1C        /* bit[2:0]=osr_p, bit[5:3]=osr_t */
#define REG_ODR             0x1D        /* bit[4:0]=odr_sel（200Hz=0, 100=1, 50=2, 25=3, 12.5=4） */
#define REG_CONFIG          0x1F        /* bit[3:1]=iir_filter（0=off, 1=1, 2=3, 3=7, ...） */
#define REG_TRIM_BASE       0x31        /* 起读 21 字节校准参数 */

#define STATUS_DRDY_PRESS   0x20        /* bit5 */
#define STATUS_DRDY_TEMP    0x40        /* bit6 */

/* 校准参数（类型与 Bosch bmp3_defs.h 一致） */
typedef struct {
    uint16_t par_t1;
    uint16_t par_t2;
    int8_t   par_t3;
    int16_t  par_p1;
    int16_t  par_p2;
    int8_t   par_p3;
    int8_t   par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t   par_p7;
    int8_t   par_p8;
    int16_t  par_p9;
    int8_t   par_p10;
    int8_t   par_p11;
    int64_t  t_lin;
} bmp388_calib_t;

struct bmp388_dev {
    i2c_master_dev_handle_t i2c_dev;
    bmp388_calib_t calib;
};

static esp_err_t write_reg(bmp388_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev->i2c_dev, buf, sizeof(buf), I2C_XFER_TIMEOUT_MS);
}

static esp_err_t read_regs(bmp388_handle_t dev, uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, out, len, I2C_XFER_TIMEOUT_MS);
}

/* 校准参数解析（Bosch bmp3.c parse_calib_data 移植） */
static void parse_calib(const uint8_t *reg_data, bmp388_calib_t *c)
{
    c->par_t1  = (uint16_t)reg_data[0] | ((uint16_t)reg_data[1] << 8);
    c->par_t2  = (uint16_t)reg_data[2] | ((uint16_t)reg_data[3] << 8);
    c->par_t3  = (int8_t)reg_data[4];
    c->par_p1  = (int16_t)((uint16_t)reg_data[5] | ((uint16_t)reg_data[6] << 8));
    c->par_p2  = (int16_t)((uint16_t)reg_data[7] | ((uint16_t)reg_data[8] << 8));
    c->par_p3  = (int8_t)reg_data[9];
    c->par_p4  = (int8_t)reg_data[10];
    c->par_p5  = (uint16_t)reg_data[11] | ((uint16_t)reg_data[12] << 8);
    c->par_p6  = (uint16_t)reg_data[13] | ((uint16_t)reg_data[14] << 8);
    c->par_p7  = (int8_t)reg_data[15];
    c->par_p8  = (int8_t)reg_data[16];
    c->par_p9  = (int16_t)((uint16_t)reg_data[17] | ((uint16_t)reg_data[18] << 8));
    c->par_p10 = (int8_t)reg_data[19];
    c->par_p11 = (int8_t)reg_data[20];
    c->t_lin = 0;
}

/* 温度补偿（Bosch compensate_temperature 移植），返回 0.01℃ */
static int64_t compensate_temp(const bmp388_calib_t *c, int64_t adc_t)
{
    int64_t p1 = adc_t - ((int64_t)256 * c->par_t1);
    int64_t p2 = (int64_t)c->par_t2 * p1;
    int64_t p3 = p1 * p1;
    int64_t p4 = p3 * c->par_t3;
    int64_t p5 = (int64_t)(p2 * 262144) + p4;
    int64_t p6 = p5 / 4294967296LL;
    /* 更新 t_lin 供气压补偿使用 */
    ((bmp388_calib_t *)c)->t_lin = p6;
    /* 温度补偿（Bosch compensate_temperature 移植），返回 0.01℃ */
    int64_t comp = (p6 * 25) / 16384;
    if (comp < -4000) {
        return -4000;                /* BMP3_MIN_TEMP_INT clamp */
    }
    if (comp > 8500) {
        return 8500;                 /* BMP3_MAX_TEMP_INT clamp */
    }
    return comp;
}

/* 气压补偿（Bosch compensate_pressure 移植），返回 0.01 Pa */
static int64_t compensate_press(const bmp388_calib_t *c, int64_t adc_p)
{
    const int64_t t_lin = c->t_lin;
    int64_t p1 = t_lin * t_lin;
    int64_t p2 = p1 / 64;
    int64_t p3 = (p2 * t_lin) / 256;
    int64_t p4 = ((int64_t)c->par_p8 * p3) / 32;
    int64_t p5 = ((int64_t)c->par_p7 * p1) * 16;
    int64_t p6 = ((int64_t)c->par_p6 * t_lin) * 4194304;
    int64_t offset = ((int64_t)c->par_p5 * 140737488355328LL) + p4 + p5 + p6;

    p2 = ((int64_t)c->par_p4 * p3) / 32;
    p4 = ((int64_t)c->par_p3 * p1) * 4;
    p5 = ((int64_t)(c->par_p2 - (int32_t)16384) * t_lin) * 2097152;
    int64_t sensitivity = ((int64_t)(c->par_p1 - (int32_t)16384) * 70368744177664LL) + p2 + p4 + p5;

    p1 = (sensitivity / 16777216) * adc_p;
    p2 = (int64_t)c->par_p10 * t_lin;
    p3 = p2 + ((int32_t)65536 * c->par_p9);
    p4 = (p3 * adc_p) / 8192;

    p5 = (adc_p * (p4 / 10)) / 512;
    p5 = p5 * 10;
    p6 = adc_p * adc_p;
    p2 = ((int64_t)c->par_p11 * p6) / 65536;
    p3 = (p2 * adc_p) / 128;
    p4 = (offset / 4) + p1 + p5 + p3;

    /* Bosch 同款：先转 uint64 再乘 25——int64 中 p4*25 会溢出（>9.2e18）导致负值！ */
    int64_t comp = (int64_t)(((uint64_t)p4 * 25) / (uint64_t)1099511627776LL);
    if (comp < 3000000) {
        return 3000000;              /* BMP3_MIN_PRES_INT clamp（300 hPa） */
    }
    if (comp > 12500000) {
        return 12500000;             /* BMP3_MAX_PRES_INT clamp（1250 hPa） */
    }
    return comp;
}

esp_err_t bmp388_init(i2c_master_bus_handle_t bus, bmp388_handle_t *out)
{
    if (bus == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = NULL;

    const uint8_t addrs[2] = { BARO_I2C_ADDR_A, BARO_I2C_ADDR_B };
    for (size_t i = 0; i < 2; i++) {
        if (i2c_master_probe(bus, addrs[i], I2C_PROBE_TIMEOUT_MS) != ESP_OK) {
            continue;
        }
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addrs[i],
            .scl_speed_hz = I2C_BUS_CLK_HZ,
        };
        bmp388_handle_t dev = calloc(1, sizeof(*dev));
        if (dev == NULL) {
            return ESP_ERR_NO_MEM;
        }
        esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &dev->i2c_dev);
        if (ret != ESP_OK) {
            free(dev);
            continue;
        }
        uint8_t chip_id = 0;
        ret = read_regs(dev, REG_CHIP_ID, &chip_id, 1);
        if (ret != ESP_OK || chip_id != BARO_CHIP_ID) {
            ESP_LOGW(TAG, "addr 0x%02X: CHIP_ID=0x%02X（期望 0x%02X）", addrs[i], chip_id, BARO_CHIP_ID);
            i2c_master_bus_rm_device(dev->i2c_dev);
            free(dev);
            continue;
        }
        /* 读取 21 字节校准参数（0x31 起） */
        uint8_t trim[21];
        ret = read_regs(dev, REG_TRIM_BASE, trim, sizeof(trim));
        if (ret != ESP_OK) {
            i2c_master_bus_rm_device(dev->i2c_dev);
            free(dev);
            continue;
        }
        parse_calib(trim, &dev->calib);
        /* 临时调试：打印原始校准字节（定位气压负值，V0.1.6 移除） */
        ESP_LOGI(TAG, "TRIM: %02x%02x%02x%02x%02x %02x%02x%02x%02x%02x %02x%02x%02x%02x%02x %02x%02x%02x%02x%02x%02x",
                 trim[0], trim[1], trim[2], trim[3], trim[4],
                 trim[5], trim[6], trim[7], trim[8], trim[9],
                 trim[10], trim[11], trim[12], trim[13], trim[14],
                 trim[15], trim[16], trim[17], trim[18], trim[19], trim[20]);
        /* 配置顺序（Bosch bmp3_set_* 移植）：先设 OSR/ODR/IIR，最后写 PWR_CTRL 进入 normal 模式 */
        write_reg(dev, REG_OSR, BARO_OSR_VAL);
        write_reg(dev, REG_ODR, BARO_ODR_VAL);
        write_reg(dev, REG_CONFIG, BARO_IIR_VAL);
        write_reg(dev, REG_PWR_CTRL, BARO_PWR_VAL);

        ESP_LOGI(TAG, "BMP388 ready @0x%02X, CHIP_ID=0x%02X", addrs[i], chip_id);
        *out = dev;
        return ESP_OK;
    }
    ESP_LOGE(TAG, "BMP388 未找到（探测 0x76/0x77 失败或 ID 不匹配）");
    return ESP_ERR_NOT_FOUND;
}

esp_err_t bmp388_read(bmp388_handle_t dev, bmp388_data_t *data)
{
    if (dev == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    /* 等数据就绪（DRDY_TEMP | DRDY_PRESS），超时 100ms */
    uint8_t status = 0;
    for (int i = 0; i < 20; i++) {
        if (read_regs(dev, REG_STATUS, &status, 1) != ESP_OK) {
            return ESP_ERR_INVALID_RESPONSE;
        }
        if (status & (STATUS_DRDY_PRESS | STATUS_DRDY_TEMP)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (!(status & (STATUS_DRDY_PRESS | STATUS_DRDY_TEMP))) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t buf[6];
    esp_err_t ret = read_regs(dev, REG_PRESS_XLSB, buf, sizeof(buf));
    if (ret != ESP_OK) {
        return ret;
    }
    /* buf: P_XLSB P_LSB P_MSB T_XLSB T_LSB T_MSB（24-bit 小端） */
    int64_t press_adc = (int64_t)buf[0] | ((int64_t)buf[1] << 8) | ((int64_t)buf[2] << 16);
    int64_t temp_adc  = (int64_t)buf[3] | ((int64_t)buf[4] << 8) | ((int64_t)buf[5] << 16);

    int64_t temp_centi = compensate_temp(&dev->calib, temp_adc);
    int64_t press_cpa  = compensate_press(&dev->calib, press_adc);

    /* 临时调试：每 5 帧打印全部原始值与校准参数（定位气压负值，V0.1.6 移除） */
    static uint8_t s_dbg_cnt = 0;
    if (++s_dbg_cnt >= 5) {
        s_dbg_cnt = 0;
        const bmp388_calib_t *c = &dev->calib;
        ESP_LOGI(TAG, "DBG raw_p=%lld raw_t=%lld t1=%u t2=%u t3=%d p1=%d p2=%d p3=%d p4=%d p5=%u p6=%u p7=%d p8=%d p9=%d p10=%d p11=%d",
                 (long long)press_adc, (long long)temp_adc,
                 c->par_t1, c->par_t2, c->par_t3,
                 c->par_p1, c->par_p2, c->par_p3, c->par_p4,
                 c->par_p5, c->par_p6, c->par_p7, c->par_p8,
                 c->par_p9, c->par_p10, c->par_p11);
    }

    data->temp_c = (float)temp_centi / 100.0f;
    data->pressure_hpa = (float)press_cpa / 100.0f / 100.0f;
    /* 标准大气公式 */
    data->altitude_m = 44330.0f * (1.0f - powf(data->pressure_hpa / 1013.25f, 0.190263f));
    return ESP_OK;
}
