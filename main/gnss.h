/*
 * gnss.h - GNSS 驱动（NEO-M8N-0-01，UART1，NMEA + UBX 双协议）
 * 引脚：TX=17 RX=18 EN(LDO)=14（高电平使能）
 * 波特率探测：[9600 → 38400 → 115200]，锁定后保持
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    bool valid;             /* 是否有可用定位数据（任意 fix） */
    bool time_valid;        /* UTC 时间有效（已同步 RTC） */
    uint8_t fix_type;       /* 0=无 1=DR 2=2D 3=3D */
    uint8_t sats;           /* 使用中的卫星数 */
    double lat;             /* 纬度（度，WGS84） */
    double lon;             /* 经度（度） */
    float alt_m;            /* 海拔（m） */
    float speed_kmh;        /* 地速（km/h） */
    float course_deg;       /* 航向（°） */
    float hdop, vdop, pdop; /* 精度因子 */
    uint32_t utc_sec;       /* UTC epoch 秒（最近一帧） */
} gnss_data_t;

/**
 * @brief 初始化：使能 LDO → UART1 → 创建解析任务（含波特率探测）
 */
esp_err_t gnss_init(void);

/** @brief 获取 GNSS 快照（线程安全） */
void gnss_get_data(gnss_data_t *out);

/** @brief 是否有有效定位 */
bool gnss_is_fixed(void);
