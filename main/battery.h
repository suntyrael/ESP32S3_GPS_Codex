/*
 * battery.h - 电池采样（ADC2 oneshot + curve fitting）+ 充电状态
 * 注意：单节锂电 1:1 分压超出 ADC 量程，需饱和保护（见 config.h BAT_SATURATION_MV）
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    float voltage_v;        /* 转换后电池电压（饱和时≈量程上限×分压比） */
    uint16_t adc_mv;        /* ADC 引脚原始电压 mV（未乘分压比） */
    int raw_count;          /* ADC 原始计数（12bit 0-4095，临时调试用） */
    uint8_t percent;        /* 电量百分比 0-100（线性近似） */
    bool saturated;         /* 是否超出 ADC 量程 */
    bool charging;          /* 充电中（CHG_SAT 开漏，低有效） */
} battery_data_t;

/**
 * @brief 初始化 ADC2 通道 + 充电状态 GPIO
 * @return ESP_OK 成功
 */
esp_err_t battery_init(void);

/** @brief 采样一次电池数据 */
esp_err_t battery_read(battery_data_t *data);
