/*
 * gnss.h - GNSS 驱动（NEO-M8N-0-01，UART1，NMEA + UBX 双协议）
 * 引脚：TX=17 RX=18 EN(LDO)=14（高电平使能）
 * 波特率探测：[9600 → 115200 → 38400]，锁定后通过 UBX-CFG-PRT 升到 115200
 *
 * 配置策略（每次上电重新下发，BBR/Flash 保存仅作加速）：
 *   CFG-PRT(115200) → MON-VER → CFG-RATE → CFG-MSG → CFG-GNSS
 *   → CFG-NAV5 → CFG-CFG(save)
 * 所有配置命令均等待 ACK/NAK（≤500ms，失败重试 1 次后降级）。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#define GNSS_MAX_SATS       32

/* 卫星系统编号（与 u-blox gnssId 对齐） */
#define GNSS_SYS_GPS        0
#define GNSS_SYS_SBAS       1
#define GNSS_SYS_GALILEO    2
#define GNSS_SYS_BEIDOU     3
#define GNSS_SYS_IMES       4
#define GNSS_SYS_QZSS       5
#define GNSS_SYS_GLONASS    6
#define GNSS_SYS_COUNT      7

/* 星座模式（对应设置页 GNSS MODE） */
typedef enum {
    GNSS_MODE_GPS_BDS = 0,      /* GPS + BeiDou（默认） */
    GNSS_MODE_ALL = 1,          /* GPS + GLONASS + BeiDou */
    GNSS_MODE_GPS_ONLY = 2,     /* GPS + SBAS + QZSS */
} gnss_mode_t;

/* 配置状态（供 UI 诊断显示） */
typedef enum {
    GNSS_CFG_PROBING = 0,       /* 波特率探测中 */
    GNSS_CFG_CONFIGURING = 1,   /* UBX 配置下发中 */
    GNSS_CFG_OK = 2,            /* 全部配置 ACK */
    GNSS_CFG_DEGRADED = 3,      /* 部分命令 NAK/超时，已降级 */
    GNSS_CFG_FALLBACK = 4,      /* UBX 不可用，仅 NMEA */
} gnss_cfg_state_t;

typedef struct {
    uint8_t sys;                /* GNSS_SYS_* */
    uint8_t prn;
    uint8_t elev;               /* 仰角 0~90 */
    uint16_t azim;              /* 方位角 0~359 */
    uint8_t cno;                /* 载噪比 dBHz */
    bool used;                  /* 是否参与当前解算（GSA/NAV-SAT） */
    uint32_t last_ms;           /* 最近更新时间 */
} gnss_sat_t;

typedef struct {
    uint8_t count;              /* 当前有效卫星条目数 */
    uint8_t used;               /* 参与解算的卫星数 */
    uint8_t gsv_total;          /* GSV 报告的可见卫星总数 */
    uint8_t sys_count[GNSS_SYS_COUNT];
    uint32_t last_update_ms;    /* 最近卫星数据更新时间 */
    uint32_t last_gsv_ms;       /* 最近 GSV/NAV-SAT 更新时间 */
    bool stale;                 /* 超过 10s 无卫星数据 */
    gnss_sat_t sats[GNSS_MAX_SATS];
} gnss_sat_info_t;

typedef struct {
    bool valid;                 /* 是否有可用定位（3D/2D 且数据未过期） */
    bool time_valid;            /* UTC 时间有效（已同步 RTC） */
    uint8_t fix_type;           /* 0=无 1=DR 2=2D 3=3D 4=GNSS+DR 5=TIME ONLY */
    uint8_t sats;               /* 使用中的卫星数（GGA/NAV-PVT） */
    uint8_t sats_tracked;       /* 跟踪中的卫星数（GSV 汇总） */
    double lat;                 /* 纬度（度，WGS84） */
    double lon;                 /* 经度（度） */
    float alt_m;                /* 海拔（MSL，m） */
    float speed_kmh;            /* 地速（km/h，瞬时） */
    float speed_avg_kmh;        /* 地速 3s 滑动平均（km/h） */
    float course_deg;           /* 水平前进方向（°） */
    float hdop, vdop, pdop;     /* 精度因子 */
    uint32_t utc_sec;           /* UTC epoch 秒（最近一帧） */

    /* ---- 诊断字段（健康页/调试页） ---- */
    uint32_t ttff_ms;           /* 上电到首次 2D/3D fix 的时间 */
    uint32_t last_fix_ms;       /* 最近一次有效 fix 的本地时间 */
    uint32_t baud;              /* 当前 UART 波特率 */
    uint8_t cfg_state;          /* gnss_cfg_state_t */
    uint8_t rate_hz;            /* 实际生效的导航刷新率 */
    bool ubx_ok;                /* UBX 配置是否可用 */
    uint16_t crc_err;           /* NMEA 校验失败计数 */
    uint16_t frames_per_s;      /* 最近 1s 有效帧数 */
    char module_sw[16];         /* MON-VER 软件版本（截断） */
} gnss_data_t;

/**
 * @brief 初始化：使能 LDO → UART1 → 创建解析/配置任务
 */
esp_err_t gnss_init(void);

/** @brief 获取 GNSS 快照（线程安全） */
void gnss_get_data(gnss_data_t *out);

/** @brief 获取卫星列表快照（线程安全，按 30s 老化过滤） */
void gnss_get_sat_info(gnss_sat_info_t *out);

/** @brief 是否有有效定位 */
bool gnss_is_fixed(void);

/** @brief 设置是否允许 GNSS 自动同步系统时钟 (RTC) */
void gnss_set_rtc_auto_sync(bool enable);
bool gnss_get_rtc_auto_sync(void);

/** @brief 获取最近接收的 NMEA 原始报文流（多行文本） */
void gnss_get_recent_log(char *out_buf, size_t max_len);

/**
 * @brief 请求修改导航刷新率（1/5/10/18 Hz）
 * 实际下发由 gnss_task 在配置就绪后执行；18Hz 若 NAK 自动回退 10Hz。
 */
void gnss_set_update_rate(uint8_t hz);

/** @brief 请求修改星座组合（gnss_mode_t） */
void gnss_set_gnss_mode(uint8_t mode);

/** @brief 当前实际生效的刷新率/星座模式 */
uint8_t gnss_get_update_rate(void);
uint8_t gnss_get_gnss_mode(void);
