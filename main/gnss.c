/*
 * gnss.c - GNSS 驱动实现（NEO-M8N，NMEA + UBX 双协议解析与配置）
 *
 * 启动流程：
 *   1. GPIO14 拉高使能 GNSS LDO → 等待模块启动
 *   2. UART1 @9600 启动，按 [9600 → 115200 → 38400] 探测波特率
 *   3. 锁定后通过 UBX-CFG-PRT 升到 115200（必须收到 ACK 才切换）
 *   4. 非阻塞配置状态机：
 *        CFG-PRT → MON-VER → CFG-RATE → CFG-MSG → CFG-GNSS
 *        → CFG-NAV5 → CFG-CFG(save)
 *      每条命令等待 ACK/NAK ≤500ms，失败重试 1 次后降级
 *   5. 解析 NMEA（RMC/GGA/GSA/GSV）与 UBX（NAV-PVT/NAV-SAT）
 *   6. 有效时间/定位 → settimeofday（UTC），UI 显示 UTC+8
 *
 * 约束：
 *   - 无源陶瓷天线，VCC_RF 电感未焊 → 不开启天线供电/CFG-ANT
 *   - NEO-M8N 最多 3 个主星座并发，4 星座会 NAK
 *   - 所有共享数据由 s_mutex 保护；解析在 gnss_task 内完成
 */
#include "gnss.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>

static const char *TAG = "gnss";

/* ==================== 参数 ==================== */
static const uint32_t GNSS_BAUDS[] = { 9600, 115200, 38400 };
#define GNSS_BAUD_CNT               (sizeof(GNSS_BAUDS) / sizeof(GNSS_BAUDS[0]))
#define GNSS_BAUD_HOLD_MS           1500    /* 每档等待时间 */
#define GNSS_LINK_LOSS_MS           10000   /* 完全无字节 10s → 重新探测 */
#define GNSS_BAUD_VERIFY_MS         2500    /* 切 115200 后 2.5s 无帧 → 回退 9600 */
#define GNSS_CFG_ACK_TIMEOUT_MS     500     /* UBX ACK/NAK 超时 */
#define GNSS_CFG_RETRY              1       /* 失败重试次数 */
#define GNSS_LINE_MAX               128
#define GNSS_LOG_RING_SIZE          14      /* NMEA 终端页显示行数 */
#define GNSS_LOG_LINE_MAX           72
#define GNSS_SAT_STALE_MS           30000   /* 单星条目老化时间 */
#define GNSS_SAT_NO_GSV_MS          10000   /* 无 GSV/NAV-SAT 判定 stale */

#define GNSS_NMEA_LOG               0       /* 串口打印原始 NMEA（调试用，默认关闭） */
#define GNSS_NMEA_LOG_EVERY         20

/* ==================== 共享状态 ==================== */
static SemaphoreHandle_t s_mutex = NULL;
static gnss_data_t s_data = { 0 };
static gnss_sat_info_t s_sat = { 0 };
static uint8_t s_gsv_total[GNSS_SYS_COUNT] = { 0 };

/* ---- UART ---- */
static QueueHandle_t s_uart_queue = NULL;
static bool s_rtc_auto_sync = true;
static uint64_t s_boot_ms = 0;
static uint64_t s_last_frame_ms = 0;    /* 最近有效 NMEA/UBX 帧 */
static uint64_t s_last_rx_ms = 0;       /* 最近收到任意字节 */
static uint64_t s_stat_ms = 0;
static uint16_t s_stat_frames = 0;
static uint32_t s_last_rtc_sync_sec = 0;
static volatile uint16_t s_cnt_gga = 0, s_cnt_rmc = 0, s_cnt_gsa = 0, s_cnt_gsv = 0;
static volatile uint16_t s_cnt_nav_pvt = 0, s_cnt_nav_sat = 0;
static uint64_t s_last_sat_log_ms = 0;
static uint64_t s_last_nmea_log_ms = 0;
static uint8_t s_seen_sys_mask = 0;

/* ---- NMEA 原始报文环形缓冲 ---- */
static char s_gnss_log_ring[GNSS_LOG_RING_SIZE][GNSS_LOG_LINE_MAX] = { 0 };
static int s_gnss_log_head = 0;
static int s_gnss_log_count = 0;

/* ---- 帧解析状态 ---- */
static uint8_t s_line[GNSS_LINE_MAX];
static size_t s_line_len = 0;
static bool s_in_ubx = false;
static uint8_t s_ubx_class, s_ubx_id;
static uint16_t s_ubx_len, s_ubx_pos;
static uint8_t s_ubx_ck_a, s_ubx_ck_b;
static uint8_t s_ubx_payload[512];

/* ==================== 3s 速度滑动平均 ==================== */
#define SPD_AVG_WIN_MS   3000
#define SPD_AVG_SLOTS    32
static uint32_t s_spd_t[SPD_AVG_SLOTS];
static float    s_spd_v[SPD_AVG_SLOTS];
static uint8_t  s_spd_cnt = 0;
static uint8_t  s_spd_idx = 0;

/* ==================== 配置状态机 ==================== */
typedef enum {
    CFG_STEP_NONE = 0,
    CFG_STEP_PRT,
    CFG_STEP_MONVER,
    CFG_STEP_RATE,
    CFG_STEP_MSG,
    CFG_STEP_GNSS,
    CFG_STEP_NAV5,
    CFG_STEP_SAVE,
    CFG_STEP_DONE,
    CFG_STEP_FALLBACK,
} cfg_step_t;

#define CFG_MSG_COUNT   6

typedef struct {
    uint8_t id;
    uint8_t res;
    uint8_t max;
    uint32_t flags;
} gnss_cfg_block_t;

static cfg_step_t s_cfg_step = CFG_STEP_NONE;
static uint64_t s_cfg_next_ms = 0;
static uint64_t s_cfg_sent_ms = 0;
static bool s_cfg_need_send = false;
static bool s_cfg_waiting_ack = false;
static uint8_t s_cfg_retry = 0;
static uint8_t s_cfg_pending_cls = 0;
static uint8_t s_cfg_pending_id = 0;
static bool s_ack_ok = false;
static uint64_t s_ack_ms = 0;
static uint64_t s_monver_ms = 0;
static uint8_t s_msg_idx = 0;
static bool s_rate_fb_done = false;
static bool s_gnss_fb_done = false;
static bool s_cfg_degraded = false;
static bool s_cfg_baud_switched = false;
static uint8_t s_effective_rate = 10;
static uint8_t s_effective_mode = GNSS_MODE_GPS_BDS;

/* 请求标志（UI 任务写入，gnss_task 消费） */
static volatile uint8_t s_req_rate_hz = 10;
static volatile uint8_t s_req_mode = GNSS_MODE_GPS_BDS;
static volatile bool s_req_rate_dirty = false;
static volatile bool s_req_mode_dirty = false;

static void parse_nmea_line(const char *line, size_t len);
static void parse_ubx_frame(void);
static void set_rtc_time(uint16_t year, uint8_t mon, uint8_t day,
                         uint8_t hour, uint8_t min, uint8_t sec);
static void cfg_tick(uint64_t now);
static void cfg_start(uint64_t now);
static void cfg_goto(cfg_step_t step, uint32_t delay_ms, uint64_t now);

static uint64_t now_ms(void)
{
    return (uint64_t)(esp_timer_get_time() / 1000);
}

static const char *gnss_mode_name(uint8_t mode)
{
    switch (mode) {
    case GNSS_MODE_GPS_BDS:  return "GPS+BeiDou";
    case GNSS_MODE_ALL:      return "GPS+GLONASS+BeiDou";
    case GNSS_MODE_GPS_ONLY: return "GPS only";
    default:                 return "unknown";
    }
}

static const char *cfg_step_name(cfg_step_t step)
{
    switch (step) {
    case CFG_STEP_PRT:    return "CFG-PRT";
    case CFG_STEP_MONVER: return "MON-VER";
    case CFG_STEP_RATE:   return "CFG-RATE";
    case CFG_STEP_MSG:    return "CFG-MSG";
    case CFG_STEP_GNSS:   return "CFG-GNSS";
    case CFG_STEP_NAV5:   return "CFG-NAV5";
    case CFG_STEP_SAVE:   return "CFG-CFG";
    default:              return "?";
    }
}

/* ==================== NMEA 工具 ==================== */
static int hex_val(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return -1;
}

/* NMEA 校验和：'$' 与 '*' 之间所有字节异或 */
static bool nmea_checksum_ok(const char *line)
{
    if (line == NULL || line[0] != '$') {
        return false;
    }
    const char *star = strchr(line, '*');
    if (star == NULL || star[1] == '\0' || star[2] == '\0') {
        return false;
    }
    uint8_t ck = 0;
    for (const char *p = line + 1; p < star; p++) {
        ck ^= (uint8_t)(*p);
    }
    int hi = hex_val(star[1]);
    int lo = hex_val(star[2]);
    if (hi < 0 || lo < 0) {
        return false;
    }
    return ck == (uint8_t)((hi << 4) | lo);
}

static double nmea_latlon(double v, char hemi)
{
    /* ddmm.mmmm -> 度；N/E 正，S/W 负 */
    int deg = (int)(v / 100.0);
    double min = v - deg * 100.0;
    double d = deg + min / 60.0;
    return (hemi == 'S' || hemi == 'W') ? -d : d;
}

static int nmea_field(const char *line, int idx, char *out, size_t sz)
{
    const char *p = line;
    int cur = 0;
    while (*p && cur < idx) {
        if (*p == ',') {
            cur++;
        }
        p++;
    }
    if (cur != idx) {
        return -1;
    }
    size_t n = 0;
    while (*p && *p != ',' && *p != '*' && n < sz - 1) {
        out[n++] = *p++;
    }
    out[n] = '\0';
    return (int)n;
}

/* 由 talker 推断 GNSS 系统；GN（组合）按 GPS 处理 */
static int talker_sys(const char *line)
{
    if (line == NULL || line[0] != '$' || line[1] != 'G') {
        return -1;
    }
    switch (line[2]) {
    case 'P': return GNSS_SYS_GPS;
    case 'L': return GNSS_SYS_GLONASS;
    case 'A': return GNSS_SYS_GALILEO;
    case 'B':
    case 'D': return GNSS_SYS_BEIDOU;
    case 'Q': return GNSS_SYS_QZSS;
    case 'N': return GNSS_SYS_GPS;
    default:  return -1;
    }
}

static void speed_avg_push(float v)
{
    uint32_t now = (uint32_t)now_ms();
    s_spd_t[s_spd_idx] = now;
    s_spd_v[s_spd_idx] = v;
    s_spd_idx = (uint8_t)((s_spd_idx + 1) % SPD_AVG_SLOTS);
    if (s_spd_cnt < SPD_AVG_SLOTS) {
        s_spd_cnt++;
    }
    float sum = 0;
    int n = 0;
    for (uint8_t i = 0; i < s_spd_cnt; i++) {
        uint8_t idx = (uint8_t)((s_spd_idx + SPD_AVG_SLOTS - 1 - i) % SPD_AVG_SLOTS);
        if (now - s_spd_t[idx] <= SPD_AVG_WIN_MS) {
            sum += s_spd_v[idx];
            n++;
        }
    }
    s_data.speed_avg_kmh = (n > 0) ? (sum / n) : v;
}

/* ==================== 卫星列表（调用方必须持有 s_mutex） ==================== */
static int sat_find(uint8_t sys, uint8_t prn)
{
    for (uint8_t i = 0; i < s_sat.count; i++) {
        if (s_sat.sats[i].sys == sys && s_sat.sats[i].prn == prn) {
            return (int)i;
        }
    }
    return -1;
}

static void sat_upsert(uint8_t sys, uint8_t prn, uint8_t elev, uint16_t azim,
                       uint8_t cno, bool set_used, bool used)
{
    if (sys >= GNSS_SYS_COUNT || prn == 0) {
        return;
    }
    int idx = sat_find(sys, prn);
    if (idx < 0) {
        if (s_sat.count >= GNSS_MAX_SATS) {
            return;
        }
        idx = s_sat.count++;
        s_sat.sats[idx].sys = sys;
        s_sat.sats[idx].prn = prn;
    }
    s_sat.sats[idx].elev = elev;
    s_sat.sats[idx].azim = azim;
    s_sat.sats[idx].cno = cno;
    s_sat.sats[idx].last_ms = (uint32_t)now_ms();
    if (set_used) {
        s_sat.sats[idx].used = used;
    }
}

static void sat_remove_sys(uint8_t sys)
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < s_sat.count; i++) {
        if (s_sat.sats[i].sys != sys) {
            s_sat.sats[n++] = s_sat.sats[i];
        }
    }
    s_sat.count = n;
    if (sys < GNSS_SYS_COUNT) {
        s_gsv_total[sys] = 0;
    }
}

static void sat_clear_used(int sys)
{
    for (uint8_t i = 0; i < s_sat.count; i++) {
        if (sys < 0 || s_sat.sats[i].sys == (uint8_t)sys) {
            s_sat.sats[i].used = false;
        }
    }
}

static void sat_set_used(int sys, uint16_t prn)
{
    for (uint8_t i = 0; i < s_sat.count; i++) {
        if ((sys < 0 || s_sat.sats[i].sys == (uint8_t)sys) && s_sat.sats[i].prn == prn) {
            s_sat.sats[i].used = true;
        }
    }
}

static void sat_recount(void)
{
    uint32_t now = (uint32_t)now_ms();
    uint8_t used = 0;
    uint16_t total = 0;

    /* 先淘汰 30s 未更新的旧条目，防止 32 槽位被幽灵卫星占满 */
    uint8_t n = 0;
    for (uint8_t i = 0; i < s_sat.count; i++) {
        if (now - s_sat.sats[i].last_ms <= GNSS_SAT_STALE_MS) {
            s_sat.sats[n++] = s_sat.sats[i];
        }
    }
    s_sat.count = n;

    memset(s_sat.sys_count, 0, sizeof(s_sat.sys_count));
    for (uint8_t i = 0; i < s_sat.count; i++) {
        if (s_sat.sats[i].used) {
            used++;
        }
        if (s_sat.sats[i].sys < GNSS_SYS_COUNT) {
            s_sat.sys_count[s_sat.sats[i].sys]++;
        }
    }
    for (int i = 0; i < GNSS_SYS_COUNT; i++) {
        total += s_gsv_total[i];
    }
    s_sat.used = used;
    s_sat.gsv_total = (uint8_t)(total > 255 ? 255 : total);
    s_sat.last_update_ms = (uint32_t)now_ms();
    s_data.sats_tracked = s_sat.gsv_total ? s_sat.gsv_total : s_sat.count;
}

/* ==================== 时间同步 ==================== */
static void set_rtc_time(uint16_t year, uint8_t mon, uint8_t day,
                         uint8_t hour, uint8_t min, uint8_t sec)
{
    if (year < 2000 || year > 2099 || mon < 1 || mon > 12 || day < 1 || day > 31 ||
        hour > 23 || min > 59 || sec > 60) {
        return;
    }
    struct tm tm = { 0 };
    tm.tm_year = year - 1900;
    tm.tm_mon = mon - 1;
    tm.tm_mday = day;
    tm.tm_hour = hour;
    tm.tm_min = min;
    tm.tm_sec = sec;
    time_t t = mktime(&tm);
    if (t == (time_t)-1) {
        return;
    }

    /* 10Hz NAV-PVT 下每秒最多同步一次，避免频繁 settimeofday */
    if (s_rtc_auto_sync && (uint32_t)t != s_last_rtc_sync_sec) {
        struct timeval tv = { .tv_sec = t, .tv_usec = 0 };
        settimeofday(&tv, NULL);
        s_last_rtc_sync_sec = (uint32_t)t;
    }

    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_data.utc_sec = (uint32_t)t;
        s_data.time_valid = true;
        xSemaphoreGive(s_mutex);
    }
}

/* ==================== NMEA 解析 ==================== */
static void log_nmea_line(const char *line, size_t len)
{
    if (line == NULL || len < 3 || line[0] != '$') {
        return;
    }
    if (s_mutex && xSemaphoreTake(s_mutex, 0) == pdTRUE) {
        size_t cpy_len = len;
        while (cpy_len > 0 && (line[cpy_len - 1] == '\r' || line[cpy_len - 1] == '\n')) {
            cpy_len--;
        }
        if (cpy_len >= GNSS_LOG_LINE_MAX) {
            cpy_len = GNSS_LOG_LINE_MAX - 1;
        }
        memcpy(s_gnss_log_ring[s_gnss_log_head], line, cpy_len);
        s_gnss_log_ring[s_gnss_log_head][cpy_len] = '\0';
        s_gnss_log_head = (s_gnss_log_head + 1) % GNSS_LOG_RING_SIZE;
        if (s_gnss_log_count < GNSS_LOG_RING_SIZE) {
            s_gnss_log_count++;
        }
        xSemaphoreGive(s_mutex);
    }
}

static void parse_rmc(const char *line)
{
    char f[20];
    char status[4] = { 0 };
    nmea_field(line, 2, status, sizeof(status));
    bool valid = (status[0] == 'A');

    if (valid) {
        /* 时间 hhmmss.ss + 日期 ddmmyy（仅有效帧同步 UTC） */
        if (nmea_field(line, 1, f, sizeof(f)) > 0 && strlen(f) >= 6) {
            uint8_t hh = (uint8_t)((f[0] - '0') * 10 + (f[1] - '0'));
            uint8_t mm = (uint8_t)((f[2] - '0') * 10 + (f[3] - '0'));
            uint8_t ss = (uint8_t)((f[4] - '0') * 10 + (f[5] - '0'));
            nmea_field(line, 9, f, sizeof(f));   /* 日期 ddmmyy */
            if (strlen(f) >= 6) {
                uint8_t day = (uint8_t)((f[0] - '0') * 10 + (f[1] - '0'));
                uint8_t mon = (uint8_t)((f[2] - '0') * 10 + (f[3] - '0'));
                uint16_t year = 2000 + (uint16_t)((f[4] - '0') * 10 + (f[5] - '0'));
                set_rtc_time(year, mon, day, hh, mm, ss);
            }
        }
    }

    char v[16] = { 0 };
    double spd = 0;
    if (nmea_field(line, 7, v, sizeof(v)) > 0) {
        spd = atof(v) * 1.852;      /* 节 → km/h */
    }

    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (valid) {
            char latf[16] = { 0 }, lonf[16] = { 0 };
            char ns[4] = { 0 }, ew[4] = { 0 };
            nmea_field(line, 3, latf, sizeof(latf));
            nmea_field(line, 4, ns, sizeof(ns));
            nmea_field(line, 5, lonf, sizeof(lonf));
            nmea_field(line, 6, ew, sizeof(ew));
            if (strlen(latf) > 0 && strlen(lonf) > 0) {
                s_data.lat = nmea_latlon(atof(latf), ns[0]);
                s_data.lon = nmea_latlon(atof(lonf), ew[0]);
            }
            char crs[16] = { 0 };
            nmea_field(line, 8, crs, sizeof(crs));
            s_data.course_deg = (float)atof(crs);
            s_data.speed_kmh = (float)spd;
            speed_avg_push((float)spd);
            if (s_data.fix_type < 2) {
                s_data.fix_type = 2;    /* RMC A = 有效定位 */
            }
            s_data.last_fix_ms = (uint32_t)now_ms();
            if (s_data.ttff_ms == 0) {
                s_data.ttff_ms = (uint32_t)(now_ms() - s_boot_ms);
            }
            s_data.valid = (s_data.fix_type >= 2);
        } else {
            s_data.speed_kmh = (float)spd;
        }
        xSemaphoreGive(s_mutex);
    }
}

static void parse_gga(const char *line)
{
    char latf[16] = { 0 }, lonf[16] = { 0 };
    char ns[4] = { 0 }, ew[4] = { 0 };
    char fix[8] = { 0 }, sats[8] = { 0 }, altf[16] = { 0 }, hdopf[16] = { 0 };
    nmea_field(line, 2, latf, sizeof(latf));
    nmea_field(line, 3, ns, sizeof(ns));
    nmea_field(line, 4, lonf, sizeof(lonf));
    nmea_field(line, 5, ew, sizeof(ew));
    nmea_field(line, 6, fix, sizeof(fix));
    nmea_field(line, 7, sats, sizeof(sats));
    nmea_field(line, 8, hdopf, sizeof(hdopf));
    nmea_field(line, 9, altf, sizeof(altf));

    int ftype = atoi(fix);
    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (strlen(latf) > 0 && strlen(lonf) > 0 && ftype > 0) {
            s_data.lat = nmea_latlon(atof(latf), ns[0]);
            s_data.lon = nmea_latlon(atof(lonf), ew[0]);
        }
        s_data.fix_type = (uint8_t)ftype;
        s_data.sats = (uint8_t)atoi(sats);
        s_data.alt_m = (float)atof(altf);
        s_data.hdop = (float)atof(hdopf);
        if (ftype >= 2) {
            s_data.last_fix_ms = (uint32_t)now_ms();
            if (s_data.ttff_ms == 0) {
                s_data.ttff_ms = (uint32_t)(now_ms() - s_boot_ms);
            }
        }
        /* GGA fix=0 时不要立刻清掉 NAV-PVT 刚给出的有效 fix，5s 老化由 getter 兜底 */
        s_data.valid = (ftype >= 2) ||
                       (s_data.last_fix_ms != 0 && (uint32_t)now_ms() - s_data.last_fix_ms < 5000);
        xSemaphoreGive(s_mutex);
    }
}

static void parse_gsa(const char *line)
{
    int sys = talker_sys(line);
    if (line[1] == 'G' && line[2] == 'N') {
        sys = -1;       /* $GNGSA 为组合星座，按 PRN 匹配所有系统 */
    }
    char pd[16] = { 0 }, hd[16] = { 0 }, vd[16] = { 0 };
    nmea_field(line, 15, pd, sizeof(pd));
    nmea_field(line, 16, hd, sizeof(hd));
    nmea_field(line, 17, vd, sizeof(vd));

    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_data.pdop = (float)atof(pd);
        s_data.hdop = (float)atof(hd);
        s_data.vdop = (float)atof(vd);

        sat_clear_used(sys);
        for (int i = 0; i < 12; i++) {
            char f[8] = { 0 };
            if (nmea_field(line, 3 + i, f, sizeof(f)) > 0) {
                sat_set_used(sys, (uint16_t)atoi(f));
            }
        }
        sat_recount();
        xSemaphoreGive(s_mutex);
    }
}

static void parse_gsv(const char *line)
{
    int sys = talker_sys(line);
    if (sys < 0) {
        return;
    }
    char f[8] = { 0 };
    int msg_num = 0, num_sv = 0;
    if (nmea_field(line, 2, f, sizeof(f)) > 0) {
        msg_num = atoi(f);
    }
    if (nmea_field(line, 3, f, sizeof(f)) > 0) {
        num_sv = atoi(f);
    }

    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint8_t bit = (uint8_t)(1U << sys);
        if ((s_seen_sys_mask & bit) == 0) {
            s_seen_sys_mask |= bit;
            static const char *names[GNSS_SYS_COUNT] = {
                "GPS", "SBAS", "Galileo", "BeiDou", "IMES", "QZSS", "GLONASS"
            };
            ESP_LOGI(TAG, "GSV system detected: %s", names[sys]);
        }
        sat_recount();      /* 先淘汰 30s 旧条目，避免 32 槽位占满 */
        if (msg_num <= 1) {
            sat_remove_sys((uint8_t)sys);
        }
        s_gsv_total[sys] = (uint8_t)(num_sv > 255 ? 255 : num_sv);

        for (int i = 0; i < 4; i++) {
            char prn[8] = { 0 }, el[8] = { 0 }, az[8] = { 0 }, cn[8] = { 0 };
            int base = 4 + i * 4;
            if (nmea_field(line, base, prn, sizeof(prn)) <= 0) {
                break;
            }
            nmea_field(line, base + 1, el, sizeof(el));
            nmea_field(line, base + 2, az, sizeof(az));
            nmea_field(line, base + 3, cn, sizeof(cn));
            sat_upsert((uint8_t)sys, (uint8_t)atoi(prn),
                       (uint8_t)atoi(el), (uint16_t)atoi(az), (uint8_t)atoi(cn),
                       false, false);
        }
        s_sat.last_gsv_ms = (uint32_t)now_ms();
        sat_recount();
        xSemaphoreGive(s_mutex);
    }
}

static void parse_nmea_line(const char *line, size_t len)
{
#if GNSS_NMEA_LOG
    static uint16_t s_nmea_cnt = 0;
    if (++s_nmea_cnt % GNSS_NMEA_LOG_EVERY == 1) {
        char dbg[96];
        size_t n = len;
        if (n > sizeof(dbg) - 1) {
            n = sizeof(dbg) - 1;
        }
        memcpy(dbg, line, n);
        dbg[n] = '\0';
        ESP_LOGI(TAG, "NMEA: %s", dbg);
    }
#else
    (void)len;
#endif
    log_nmea_line(line, strlen(line));

    /* 首次锁定后打印少量原始 NMEA，便于确认模块实际输出哪些语句 */
    static uint8_t s_nmea_dbg_left = 8;
    if (s_nmea_dbg_left > 0) {
        s_nmea_dbg_left--;
        ESP_LOGI(TAG, "NMEA[%u]: %s", (unsigned)(8 - s_nmea_dbg_left), line);
    }

    if (!nmea_checksum_ok(line)) {
        static uint8_t s_crc_dbg_left = 3;
        if (s_crc_dbg_left > 0) {
            s_crc_dbg_left--;
            ESP_LOGW(TAG, "NMEA checksum failed: %s", line);
        }
        if (s_mutex && xSemaphoreTake(s_mutex, 0) == pdTRUE) {
            if (s_data.crc_err < 0xFFFF) {
                s_data.crc_err++;
            }
            xSemaphoreGive(s_mutex);
        }
        return;
    }

    const char *msg = line + 3;     /* $GPRMC → "RMC" */
    if (strncmp(msg, "RMC", 3) == 0) {
        s_cnt_rmc++;
        parse_rmc(line);
    } else if (strncmp(msg, "GGA", 3) == 0) {
        s_cnt_gga++;
        parse_gga(line);
    } else if (strncmp(msg, "GSA", 3) == 0) {
        s_cnt_gsa++;
        parse_gsa(line);
    } else if (strncmp(msg, "GSV", 3) == 0) {
        s_cnt_gsv++;
        parse_gsv(line);
    }
}

/* ==================== UBX 解析 ==================== */
static void parse_monver(void)
{
    if (s_ubx_len < 30) {
        return;
    }
    char sw[16] = { 0 };
    char hw[11] = { 0 };
    size_t n = s_ubx_len < 15 ? s_ubx_len : 15;
    memcpy(sw, s_ubx_payload, n);
    sw[n] = '\0';
    if (s_ubx_len >= 40) {
        memcpy(hw, s_ubx_payload + 30, 10);
        hw[10] = '\0';
    }

    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        memcpy(s_data.module_sw, sw, sizeof(s_data.module_sw));
        s_data.module_sw[sizeof(s_data.module_sw) - 1] = '\0';
        xSemaphoreGive(s_mutex);
    }
    s_monver_ms = now_ms();
    ESP_LOGI(TAG, "MON-VER: SW=%s HW=%s", sw, hw[0] ? hw : "?");
}

static void parse_nav_pvt(void)
{
    const uint8_t *p = s_ubx_payload;
    if (s_ubx_len < 92) {
        return;
    }
    uint16_t year = (uint16_t)(p[4] | (p[5] << 8));
    uint8_t mon = p[6], day = p[7], hour = p[8], min = p[9], sec = p[10];
    uint8_t valid = p[11];
    uint8_t fix_type = p[20];
    uint8_t flags = p[21];
    uint8_t num_sv = p[23];
    int32_t lon = (int32_t)(p[24] | (p[25] << 8) | ((uint32_t)p[26] << 16) | ((uint32_t)p[27] << 24));
    int32_t lat = (int32_t)(p[28] | (p[29] << 8) | ((uint32_t)p[30] << 16) | ((uint32_t)p[31] << 24));
    int32_t hmsl = (int32_t)(p[36] | (p[37] << 8) | ((uint32_t)p[38] << 16) | ((uint32_t)p[39] << 24));
    int32_t g_speed = (int32_t)(p[60] | (p[61] << 8) | ((uint32_t)p[62] << 16) | ((uint32_t)p[63] << 24));
    int32_t head_mot = (int32_t)(p[64] | (p[65] << 8) | ((uint32_t)p[66] << 16) | ((uint32_t)p[67] << 24));
    uint16_t p_dop = (uint16_t)(p[76] | (p[77] << 8));

    bool date_ok = (valid & 0x01) != 0;
    bool time_ok = (valid & 0x02) != 0;
    bool fix_ok = (flags & 0x01) != 0;

    /* time-only fix（fixType=5）也允许对时 */
    if (date_ok && time_ok && year >= 2000 && year <= 2099) {
        set_rtc_time(year, mon, day, hour, min, sec);
    }

    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_data.fix_type = fix_type;
        s_data.sats = num_sv;
        if (fix_type >= 2 && fix_ok) {
            s_data.lat = lat / 1e7;
            s_data.lon = lon / 1e7;
            s_data.alt_m = hmsl / 1000.0f;
            s_data.speed_kmh = g_speed * 0.0036f;       /* mm/s → km/h */
            speed_avg_push(s_data.speed_kmh);
            float crs = head_mot / 1e5f;
            while (crs < 0.0f) {
                crs += 360.0f;
            }
            while (crs >= 360.0f) {
                crs -= 360.0f;
            }
            s_data.course_deg = crs;
            s_data.pdop = p_dop / 100.0f;
            s_data.last_fix_ms = (uint32_t)now_ms();
            if (s_data.ttff_ms == 0) {
                s_data.ttff_ms = (uint32_t)(now_ms() - s_boot_ms);
            }
            s_data.valid = true;
        } else {
            s_data.valid = false;
        }
        xSemaphoreGive(s_mutex);
    }
}

static void parse_nav_sat(void)
{
    const uint8_t *p = s_ubx_payload;
    if (s_ubx_len < 8) {
        return;
    }
    uint8_t num = p[5];
    if (num > GNSS_MAX_SATS) {
        num = GNSS_MAX_SATS;
    }

    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        sat_recount();      /* 先淘汰 30s 旧条目 */
        for (uint8_t i = 0; i < num; i++) {
            int b = 8 + i * 12;
            if (b + 12 > s_ubx_len) {
                break;
            }
            uint8_t sys = p[b];
            uint8_t sv_id = p[b + 1];
            uint8_t cno = p[b + 2];
            uint8_t elev = p[b + 3];
            uint16_t azim = (uint16_t)(p[b + 4] | (p[b + 5] << 8));
            uint16_t prn = (uint16_t)(p[b + 6] | (p[b + 7] << 8));
            uint32_t flags = (uint32_t)p[b + 8] | ((uint32_t)p[b + 9] << 8) |
                             ((uint32_t)p[b + 10] << 16) | ((uint32_t)p[b + 11] << 24);
            if (prn == 0) {
                prn = sv_id;
            }
            sat_upsert(sys, (uint8_t)(prn > 255 ? 255 : prn), elev, azim,
                       cno, true, (flags & 0x08) != 0);
        }
        s_sat.last_gsv_ms = (uint32_t)now_ms();
        sat_recount();
        xSemaphoreGive(s_mutex);
    }
}

static void parse_ubx_frame(void)
{
    uint64_t now = now_ms();

    if (s_ubx_class == 0x05) {          /* ACK-ACK / ACK-NAK */
        if (s_ubx_len >= 2 && s_cfg_waiting_ack) {
            uint8_t cls = s_ubx_payload[0];
            uint8_t id = s_ubx_payload[1];
            if (cls == s_cfg_pending_cls && id == s_cfg_pending_id) {
                s_ack_ok = (s_ubx_id == 0x01);
                s_ack_ms = now;
            }
        }
        return;
    }
    if (s_ubx_class == 0x0A && s_ubx_id == 0x04) {   /* MON-VER */
        parse_monver();
        return;
    }
    if (s_ubx_class == 0x01 && s_ubx_id == 0x07) {   /* NAV-PVT */
        s_cnt_nav_pvt++;
        parse_nav_pvt();
        return;
    }
    if (s_ubx_class == 0x01 && s_ubx_id == 0x35) {   /* NAV-SAT（可选） */
        s_cnt_nav_sat++;
        parse_nav_sat();
    }
}

/* ==================== 字节流组帧 ==================== */
static void feed_byte(uint8_t b)
{
    s_last_rx_ms = now_ms();

    if (s_in_ubx) {
        /* UBX 帧：sync(2) class id len(2) payload(len) CK_A CK_B */
        if (s_ubx_pos < 4) {
            if (s_ubx_pos == 0) {
                s_ubx_class = b;
                s_ubx_ck_a = s_ubx_class;
                s_ubx_ck_b = s_ubx_ck_a;
            } else if (s_ubx_pos == 1) {
                s_ubx_id = b;
                s_ubx_ck_a += s_ubx_id;
                s_ubx_ck_b += s_ubx_ck_a;
            } else if (s_ubx_pos == 2) {
                s_ubx_len = b;
                s_ubx_ck_a += b;
                s_ubx_ck_b += s_ubx_ck_a;
            } else {
                s_ubx_len |= (uint16_t)b << 8;
                s_ubx_ck_a += b;
                s_ubx_ck_b += s_ubx_ck_a;
                if (s_ubx_len > sizeof(s_ubx_payload)) {
                    s_in_ubx = false;
                }
            }
        } else if (s_ubx_pos < 4 + s_ubx_len) {
            s_ubx_payload[s_ubx_pos - 4] = b;
            s_ubx_ck_a += b;
            s_ubx_ck_b += s_ubx_ck_a;
        } else if (s_ubx_pos == 4 + s_ubx_len) {
            if (b != s_ubx_ck_a) {
                s_in_ubx = false;
                return;
            }
        } else {
            if (b == s_ubx_ck_b) {
                s_last_frame_ms = now_ms();
                s_stat_frames++;
                parse_ubx_frame();
            }
            s_in_ubx = false;
            return;
        }
        s_ubx_pos++;
        return;
    }

    if (s_line_len == 0 && b == '$') {
        s_line[0] = '$';
        s_line_len = 1;
        return;
    }
    if (s_line_len == 0 && b == 0xB5) {
        s_in_ubx = true;
        s_ubx_pos = 0;
        s_ubx_len = 0;
        return;
    }
    if (s_line_len > 0) {
        if (b == '\n') {
            s_line[s_line_len] = '\0';
            s_last_frame_ms = now_ms();
            s_stat_frames++;
            parse_nmea_line((const char *)s_line, s_line_len);
            s_line_len = 0;
        } else if (b == 0xB5) {
            s_in_ubx = true;
            s_ubx_pos = 0;
            s_ubx_len = 0;
            s_line_len = 0;
        } else if (s_line_len < GNSS_LINE_MAX - 1) {
            s_line[s_line_len++] = b;
        } else {
            s_line_len = 0;    /* 行过长，丢弃 */
        }
    }
}

/* ==================== UBX 发送 ==================== */
static bool ubx_send(uint8_t class_id, uint8_t msg_id, const uint8_t *payload, uint16_t len)
{
    uint8_t buf[128];
    if (len + 8 > sizeof(buf)) {
        return false;
    }
    buf[0] = 0xB5;
    buf[1] = 0x62;
    buf[2] = class_id;
    buf[3] = msg_id;
    buf[4] = len & 0xFF;
    buf[5] = len >> 8;
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 0; i < len; i++) {
        buf[6 + i] = payload[i];
        ck_a += payload[i];
        ck_b += ck_a;
    }
    ck_a += class_id + msg_id + buf[4] + buf[5];
    ck_b += ck_a;
    buf[6 + len] = ck_a;
    buf[7 + len] = ck_b;
    int written = uart_write_bytes(UART_NUM_1, buf, len + 8);
    return written == (int)(len + 8);
}

/* ==================== 配置状态机 ==================== */
static void cfg_send_cmd(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len, uint64_t now)
{
    s_cfg_pending_cls = cls;
    s_cfg_pending_id = id;
    s_cfg_waiting_ack = true;
    s_cfg_sent_ms = now;
    s_ack_ok = false;
    s_ack_ms = 0;
    if (!ubx_send(cls, id, payload, len)) {
        ESP_LOGW(TAG, "UBX send failed: %02X/%02X", cls, id);
    }
}

static void cfg_goto(cfg_step_t step, uint32_t delay_ms, uint64_t now)
{
    s_cfg_step = step;
    s_cfg_next_ms = now + delay_ms;
    s_cfg_need_send = true;
    s_cfg_waiting_ack = false;
    s_cfg_retry = 0;
}

static void cfg_finish(uint64_t now)
{
    (void)now;
    s_cfg_step = CFG_STEP_DONE;
    s_data.rate_hz = s_effective_rate;
    s_data.ubx_ok = true;
    s_data.cfg_state = s_cfg_degraded ? GNSS_CFG_DEGRADED : GNSS_CFG_OK;
    ESP_LOGI(TAG, "UBX config %s: rate=%uHz mode=%s baud=%lu",
             s_cfg_degraded ? "DEGRADED" : "OK",
             (unsigned)s_effective_rate, gnss_mode_name(s_effective_mode),
             (unsigned long)s_data.baud);
}

static void cfg_fallback(uint64_t now)
{
    (void)now;
    s_cfg_step = CFG_STEP_FALLBACK;
    s_data.ubx_ok = false;
    s_data.cfg_state = GNSS_CFG_FALLBACK;
    ESP_LOGW(TAG, "UBX unavailable, fallback to NMEA-only @%lu", (unsigned long)s_data.baud);
}

static void cfg_send_current(uint64_t now)
{
    switch (s_cfg_step) {
    case CFG_STEP_PRT: {
        /* portID, reserved, txReady, mode=0x000008D0(8N1), baud=0x0001C200,
         * inProto=UBX+NMEA, outProto=NMEA+UBX, flags, reserved */
        const uint8_t p[20] = {
            0x01, 0x00,
            0x00, 0x00,
            0xD0, 0x08, 0x00, 0x00,
            0x00, 0xC2, 0x01, 0x00,
            0x03, 0x00,
            0x03, 0x00,
            0x00, 0x00,
            0x00, 0x00,
        };
        ESP_LOGI(TAG, "CFG-PRT -> UART1 115200 8N1 (wait ACK)");
        cfg_send_cmd(0x06, 0x00, p, sizeof(p), now);
        break;
    }
    case CFG_STEP_MONVER:
        ESP_LOGI(TAG, "poll MON-VER (wait response)");
        ubx_send(0x0A, 0x04, NULL, 0);
        s_cfg_sent_ms = now;
        s_cfg_waiting_ack = false;
        break;
    case CFG_STEP_RATE: {
        uint16_t meas = (uint16_t)(1000U / (s_effective_rate ? s_effective_rate : 10));
        uint8_t p[6] = {
            (uint8_t)(meas & 0xFF), (uint8_t)(meas >> 8),
            0x01, 0x00,     /* navRate = 1 */
            0x01, 0x00,     /* timeRef = GPS */
        };
        ESP_LOGI(TAG, "CFG-RATE request: %u Hz (%u ms)", (unsigned)s_effective_rate, (unsigned)meas);
        cfg_send_cmd(0x06, 0x08, p, sizeof(p), now);
        break;
    }
    case CFG_STEP_MSG: {
        static const uint8_t msg_cls[CFG_MSG_COUNT] = { 0x01, 0x01, 0xF0, 0xF0, 0xF0, 0xF0 };
        static const uint8_t msg_id[CFG_MSG_COUNT]  = { 0x07, 0x35, 0x00, 0x04, 0x02, 0x03 };
        uint8_t nmea_rate = (uint8_t)((s_effective_rate + 1) / 2);
        if (nmea_rate < 1) {
            nmea_rate = 1;
        }
        if (s_msg_idx >= CFG_MSG_COUNT) {
            cfg_goto(CFG_STEP_GNSS, 0, now);
            break;
        }
        uint8_t rate = (s_msg_idx == 0) ? 1 : nmea_rate;   /* NAV-PVT 每个解算都发 */
        static const char *msg_name[CFG_MSG_COUNT] = { "NAV-PVT", "NAV-SAT", "GGA", "RMC", "GSA", "GSV" };
        ESP_LOGI(TAG, "CFG-MSG: %s UART1 rate=%u", msg_name[s_msg_idx], (unsigned)rate);
        uint8_t p[8] = { msg_cls[s_msg_idx], msg_id[s_msg_idx], 0, rate, 0, 0, 0, 0 };
        cfg_send_cmd(0x06, 0x01, p, sizeof(p), now);
        break;
    }
    case CFG_STEP_GNSS: {
        /* NEO-M8N 最多 3 个主星座；SBAS/QZSS 在国内无收益且占通道，默认不启用 */
        static const gnss_cfg_block_t gps_bds[] = {
            { GNSS_SYS_GPS,     8, 16, 0x01 },
            { GNSS_SYS_BEIDOU,  8, 16, 0x01 },
        };
        static const gnss_cfg_block_t all_gnss[] = {
            { GNSS_SYS_GPS,     8, 16, 0x01 },
            { GNSS_SYS_GLONASS, 8, 14, 0x01 },
            { GNSS_SYS_BEIDOU,  8, 16, 0x01 },
        };
        static const gnss_cfg_block_t gps_only[] = {
            { GNSS_SYS_GPS,     8, 16, 0x01 },
        };
        const gnss_cfg_block_t *blocks = gps_bds;
        uint8_t n = sizeof(gps_bds) / sizeof(gps_bds[0]);
        if (s_effective_mode == GNSS_MODE_ALL) {
            blocks = all_gnss;
            n = sizeof(all_gnss) / sizeof(all_gnss[0]);
        } else if (s_effective_mode == GNSS_MODE_GPS_ONLY) {
            blocks = gps_only;
            n = sizeof(gps_only) / sizeof(gps_only[0]);
        }
        uint8_t p[4 + 8 * 5] = { 0 };
        p[0] = 0x00;        /* version */
        p[1] = 32;          /* numTrkChHw */
        p[2] = 32;          /* numTrkChUse */
        p[3] = n;
        for (uint8_t i = 0; i < n; i++) {
            p[4 + i * 8 + 0] = blocks[i].id;
            p[4 + i * 8 + 1] = blocks[i].res;
            p[4 + i * 8 + 2] = blocks[i].max;
            p[4 + i * 8 + 3] = 0;
            p[4 + i * 8 + 4] = (uint8_t)(blocks[i].flags & 0xFF);
            p[4 + i * 8 + 5] = (uint8_t)((blocks[i].flags >> 8) & 0xFF);
            p[4 + i * 8 + 6] = (uint8_t)((blocks[i].flags >> 16) & 0xFF);
            p[4 + i * 8 + 7] = (uint8_t)((blocks[i].flags >> 24) & 0xFF);
        }
        ESP_LOGI(TAG, "CFG-GNSS request: %s (%u blocks)", gnss_mode_name(s_effective_mode), (unsigned)n);
        cfg_send_cmd(0x06, 0x3E, p, (uint16_t)(4 + 8 * n), now);
        break;
    }
    case CFG_STEP_NAV5: {
        uint8_t p[36] = { 0 };
        p[0] = 0x05;        /* mask: dyn + fixMode */
        p[1] = 0x00;
        p[2] = 0x00;        /* dynModel = portable */
        p[3] = 0x03;        /* fixMode = auto 2D/3D */
        ESP_LOGI(TAG, "CFG-NAV5 request: portable + auto 2D/3D");
        cfg_send_cmd(0x06, 0x24, p, sizeof(p), now);
        break;
    }
    case CFG_STEP_SAVE: {
        uint8_t p[13] = { 0 };
        p[4] = 0xFF;        /* saveMask = 0x0000FFFF */
        p[5] = 0xFF;
        p[12] = 0x03;       /* deviceMask = BBR + Flash */
        ESP_LOGI(TAG, "CFG-CFG save BBR+Flash (wait ACK)");
        cfg_send_cmd(0x06, 0x09, p, sizeof(p), now);
        break;
    }
    default:
        break;
    }
}

static void cfg_on_success(uint64_t now)
{
    switch (s_cfg_step) {
    case CFG_STEP_PRT:
        uart_set_baudrate(UART_NUM_1, 115200);
        uart_flush_input(UART_NUM_1);
        s_data.baud = 115200;
        s_cfg_baud_switched = true;
        ESP_LOGI(TAG, "UBX CFG-PRT ACK: switched to 115200");
        cfg_goto(CFG_STEP_MONVER, 100, now);
        break;
    case CFG_STEP_MONVER:
        s_cfg_baud_switched = false;    /* 已收到 115200 下的 UBX 响应，链路确认成功 */
        cfg_goto(CFG_STEP_RATE, 0, now);
        break;
    case CFG_STEP_RATE:
        s_msg_idx = 0;
        ESP_LOGI(TAG, "UBX CFG-RATE ACK: %u Hz", (unsigned)s_effective_rate);
        cfg_goto(CFG_STEP_MSG, 0, now);
        break;
    case CFG_STEP_MSG:
        s_msg_idx++;
        if (s_msg_idx < CFG_MSG_COUNT) {
            cfg_goto(CFG_STEP_MSG, 0, now);
        } else {
            ESP_LOGI(TAG, "CFG-MSG done: NAV-PVT=1, NMEA GGA/RMC/GSA/GSV=%u",
                     (unsigned)((s_effective_rate + 1) / 2));
            cfg_goto(CFG_STEP_GNSS, 0, now);
        }
        break;
    case CFG_STEP_GNSS:
        ESP_LOGI(TAG, "UBX CFG-GNSS ACK: %s", gnss_mode_name(s_effective_mode));
        cfg_goto(CFG_STEP_NAV5, 0, now);
        break;
    case CFG_STEP_NAV5:
        ESP_LOGI(TAG, "UBX CFG-NAV5 ACK");
        cfg_goto(CFG_STEP_SAVE, 0, now);
        break;
    case CFG_STEP_SAVE:
        cfg_finish(now);
        break;
    default:
        break;
    }
}

static void cfg_on_failure(uint64_t now, bool timeout)
{
    ESP_LOGW(TAG, "UBX %s %s", cfg_step_name(s_cfg_step), timeout ? "timeout" : "NAK");

    switch (s_cfg_step) {
    case CFG_STEP_PRT:
        cfg_fallback(now);
        break;
    case CFG_STEP_MONVER:
        s_cfg_baud_switched = false;    /* 已收到 115200 下的 UBX 响应，链路确认成功 */
        cfg_goto(CFG_STEP_RATE, 0, now);
        break;
    case CFG_STEP_RATE:
        if (s_req_rate_hz != 10 && !s_rate_fb_done) {
            s_rate_fb_done = true;
            s_effective_rate = 10;
            s_cfg_degraded = true;
            ESP_LOGW(TAG, "rate %uHz not supported, fallback 10Hz", (unsigned)s_req_rate_hz);
            cfg_goto(CFG_STEP_RATE, 0, now);
        } else {
            s_cfg_degraded = true;
            cfg_goto(CFG_STEP_MSG, 0, now);
        }
        break;
    case CFG_STEP_MSG: {
        static const char *msg_name[CFG_MSG_COUNT] = { "NAV-PVT", "NAV-SAT", "GGA", "RMC", "GSA", "GSV" };
        ESP_LOGW(TAG, "CFG-MSG %s not accepted, skip", msg_name[s_msg_idx]);
        s_cfg_degraded = true;
        s_msg_idx++;
        if (s_msg_idx < CFG_MSG_COUNT) {
            cfg_goto(CFG_STEP_MSG, 0, now);
        } else {
            cfg_goto(CFG_STEP_GNSS, 0, now);
        }
        break;
    }
    case CFG_STEP_GNSS:
        if (s_req_mode != GNSS_MODE_GPS_BDS && !s_gnss_fb_done) {
            s_gnss_fb_done = true;
            s_effective_mode = GNSS_MODE_GPS_BDS;
            s_cfg_degraded = true;
            ESP_LOGW(TAG, "GNSS mode %u NAK, fallback GPS+BDS", (unsigned)s_req_mode);
            cfg_goto(CFG_STEP_GNSS, 0, now);
        } else {
            s_cfg_degraded = true;
            cfg_goto(CFG_STEP_NAV5, 0, now);
        }
        break;
    case CFG_STEP_NAV5:
        s_cfg_degraded = true;
        cfg_goto(CFG_STEP_SAVE, 0, now);
        break;
    case CFG_STEP_SAVE:
        cfg_finish(now);
        break;
    default:
        break;
    }
}

static void cfg_tick(uint64_t now)
{
    if (s_cfg_step == CFG_STEP_NONE || s_cfg_step == CFG_STEP_DONE ||
        s_cfg_step == CFG_STEP_FALLBACK) {
        return;
    }
    if (now < s_cfg_next_ms) {
        return;
    }
    if (s_cfg_need_send) {
        s_cfg_need_send = false;
        cfg_send_current(now);
        return;
    }

    if (s_cfg_step == CFG_STEP_MONVER) {
        if (s_monver_ms >= s_cfg_sent_ms) {
            cfg_on_success(now);
        } else if (now - s_cfg_sent_ms > GNSS_CFG_ACK_TIMEOUT_MS) {
            cfg_on_failure(now, true);
        }
        return;
    }

    if (!s_cfg_waiting_ack) {
        return;
    }
    if (s_ack_ms >= s_cfg_sent_ms) {
        s_cfg_waiting_ack = false;
        if (s_ack_ok) {
            cfg_on_success(now);
        } else {
            cfg_on_failure(now, false);
        }
        return;
    }
    if (now - s_cfg_sent_ms > GNSS_CFG_ACK_TIMEOUT_MS) {
        s_cfg_waiting_ack = false;
        if (s_cfg_retry < GNSS_CFG_RETRY) {
            s_cfg_retry++;
            s_cfg_need_send = true;
            s_cfg_next_ms = now;
        } else {
            cfg_on_failure(now, true);
        }
    }
}

static void cfg_start(uint64_t now)
{
    s_cfg_degraded = false;
    s_rate_fb_done = false;
    s_gnss_fb_done = false;
    s_msg_idx = 0;
    s_cfg_baud_switched = false;
    s_effective_rate = s_req_rate_hz;
    s_effective_mode = s_req_mode;
    s_data.cfg_state = GNSS_CFG_CONFIGURING;
    if (s_data.baud == 115200) {
        cfg_goto(CFG_STEP_MONVER, 0, now);
    } else {
        cfg_goto(CFG_STEP_PRT, 0, now);
    }
}

/* ==================== 周期诊断日志 ==================== */
static void log_sat_summary(void)
{
    if (s_mutex == NULL || xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }
    uint8_t max_cno = 0;
    for (uint8_t i = 0; i < s_sat.count; i++) {
        if (s_sat.sats[i].cno > max_cno) {
            max_cno = s_sat.sats[i].cno;
        }
    }
    uint32_t now = (uint32_t)now_ms();
    if (s_sat.last_gsv_ms == 0 || now - s_sat.last_gsv_ms > GNSS_SAT_NO_GSV_MS) {
        ESP_LOGW(TAG, "SAT: no GSV/NAV-SAT data (entries=%u gsv_total=%u), check NMEA GSV / CFG-MSG",
                 (unsigned)s_sat.count, (unsigned)s_sat.gsv_total);
    } else {
        ESP_LOGI(TAG, "SAT: used=%u view=%u GPS=%u GLO=%u BDS=%u GAL=%u QZSS=%u maxCN0=%u GSV=%u NAV-SAT=%u",
                 (unsigned)s_sat.used,
                 (unsigned)(s_sat.gsv_total ? s_sat.gsv_total : s_sat.count),
                 (unsigned)s_sat.sys_count[GNSS_SYS_GPS],
                 (unsigned)s_sat.sys_count[GNSS_SYS_GLONASS],
                 (unsigned)s_sat.sys_count[GNSS_SYS_BEIDOU],
                 (unsigned)s_sat.sys_count[GNSS_SYS_GALILEO],
                 (unsigned)s_sat.sys_count[GNSS_SYS_QZSS],
                 (unsigned)max_cno,
                 (unsigned)s_cnt_gsv, (unsigned)s_cnt_nav_sat);
    }
    xSemaphoreGive(s_mutex);
}

/* ==================== 主任务 ==================== */
static void gnss_task(void *arg)
{
    (void)arg;
    uint32_t baud_idx = 0;
    bool locked = false;
    uint64_t probe_start = now_ms();

    /* 等待模块上电稳定（dev_note §8：≥100ms） */
    vTaskDelay(pdMS_TO_TICKS(150));

    uart_set_baudrate(UART_NUM_1, GNSS_BAUDS[baud_idx]);
    uart_flush_input(UART_NUM_1);
    s_data.baud = GNSS_BAUDS[baud_idx];
    s_data.cfg_state = GNSS_CFG_PROBING;
    ESP_LOGI(TAG, "baud probe start: attempt 1/%u @%lu", (unsigned)GNSS_BAUD_CNT,
             (unsigned long)GNSS_BAUDS[baud_idx]);

    uint8_t buf[512];
    for (;;) {
        uint64_t now = now_ms();

        if (!locked) {
            if (s_last_frame_ms == 0) {
                if (now - probe_start > GNSS_BAUD_HOLD_MS) {
                    probe_start = now;
                    baud_idx = (baud_idx + 1) % GNSS_BAUD_CNT;
                    uart_set_baudrate(UART_NUM_1, GNSS_BAUDS[baud_idx]);
                    uart_flush_input(UART_NUM_1);
                    s_data.baud = GNSS_BAUDS[baud_idx];
                    ESP_LOGI(TAG, "no data, try attempt %u/%u @%lu",
                             (unsigned)(baud_idx + 1), (unsigned)GNSS_BAUD_CNT,
                             (unsigned long)GNSS_BAUDS[baud_idx]);
                }
            } else {
                locked = true;
                s_data.baud = GNSS_BAUDS[baud_idx];
                ESP_LOGI(TAG, "locked @%lu after %lums (attempt %u/%u)",
                         (unsigned long)s_data.baud,
                         (unsigned long)(now - s_boot_ms),
                         (unsigned)(baud_idx + 1), (unsigned)GNSS_BAUD_CNT);
                cfg_start(now);
            }
        } else {
            /* 完全失联 → 重新探测 */
            if (now - s_last_rx_ms > GNSS_LINK_LOSS_MS) {
                ESP_LOGW(TAG, "GNSS link lost, re-probing");
                locked = false;
                baud_idx = 0;
                probe_start = now;
                s_last_frame_ms = 0;
                s_last_rx_ms = now;
                s_cfg_step = CFG_STEP_NONE;
                s_cfg_baud_switched = false;
                s_data.ubx_ok = false;
                s_data.cfg_state = GNSS_CFG_PROBING;
                uart_set_baudrate(UART_NUM_1, GNSS_BAUDS[baud_idx]);
                uart_flush_input(UART_NUM_1);
            } else {
                /* 切 115200 后长时间无帧 → 回退 9600，NMEA-only 兜底 */
                if (s_cfg_baud_switched && now - s_last_frame_ms > GNSS_BAUD_VERIFY_MS) {
                    ESP_LOGW(TAG, "no frame after 115200 switch, fallback 9600");
                    uart_set_baudrate(UART_NUM_1, 9600);
                    uart_flush_input(UART_NUM_1);
                    s_data.baud = 9600;
                    s_cfg_baud_switched = false;
                    cfg_fallback(now);
                } else {
                    cfg_tick(now);

                    /* 配置完成后处理 UI 请求的速率/星座变更 */
                    if (s_cfg_step == CFG_STEP_DONE) {
                        if (s_req_rate_dirty) {
                            s_req_rate_dirty = false;
                            s_cfg_degraded = false;
                            s_rate_fb_done = false;
                            s_effective_rate = s_req_rate_hz;
                            s_msg_idx = 0;
                            cfg_goto(CFG_STEP_RATE, 0, now);
                        } else if (s_req_mode_dirty) {
                            s_req_mode_dirty = false;
                            s_cfg_degraded = false;
                            s_gnss_fb_done = false;
                            s_effective_mode = s_req_mode;
                            cfg_goto(CFG_STEP_GNSS, 0, now);
                        }
                    }
                }
            }
        }

        int len = uart_read_bytes(UART_NUM_1, buf, sizeof(buf), pdMS_TO_TICKS(100));
        for (int i = 0; i < len; i++) {
            feed_byte(buf[i]);
        }

        /* 帧率与 NMEA/UBX 计数统计 */
        if (now - s_stat_ms >= 1000) {
            s_data.frames_per_s = s_stat_frames;
            s_stat_frames = 0;
            s_data.nmea_gga = s_cnt_gga;
            s_data.nmea_rmc = s_cnt_rmc;
            s_data.nmea_gsa = s_cnt_gsa;
            s_data.nmea_gsv = s_cnt_gsv;
            s_data.ubx_nav_pvt = s_cnt_nav_pvt;
            s_data.ubx_nav_sat = s_cnt_nav_sat;
            s_stat_ms = now;
        }

        /* 周期打印卫星/串口统计，便于无屏幕时定位“为什么卫星页没数据” */
        if (locked && now - s_last_sat_log_ms >= 5000) {
            s_last_sat_log_ms = now;
            log_sat_summary();
        }
        if (now - s_last_nmea_log_ms >= 10000) {
            s_last_nmea_log_ms = now;
            ESP_LOGI(TAG, "COUNTERS: GGA=%u RMC=%u GSA=%u GSV=%u | NAV-PVT=%u NAV-SAT=%u | crc=%u | cfg_step=%s",
                     (unsigned)s_cnt_gga, (unsigned)s_cnt_rmc, (unsigned)s_cnt_gsa, (unsigned)s_cnt_gsv,
                     (unsigned)s_cnt_nav_pvt, (unsigned)s_cnt_nav_sat,
                     (unsigned)s_data.crc_err, cfg_step_name(s_cfg_step));
        }
    }
}

/* ==================== 接口 ==================== */
esp_err_t gnss_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_boot_ms = now_ms();
    s_stat_ms = s_boot_ms;
    s_data.rate_hz = 10;

    /* EN：LDO 使能（高电平） */
    gpio_config_t en = {
        .pin_bit_mask = (1ULL << PIN_GNSS_LDO_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&en), TAG, "gpio_config(EN) failed");
    gpio_set_level(PIN_GNSS_LDO_EN, 1);

    /* UART1：TX17/RX18，RX ring 4KB 以承受 10Hz NAV + 多星座 NMEA 突发 */
    uart_config_t uart_cfg = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(UART_NUM_1, 4096, 0, 20, &s_uart_queue, 0),
                        TAG, "uart_driver_install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(UART_NUM_1, &uart_cfg), TAG, "uart_param_config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(UART_NUM_1, PIN_GNSS_TX, PIN_GNSS_RX, -1, -1),
                        TAG, "uart_set_pin failed");

    xTaskCreate(gnss_task, "gnss_task", TASK_STACK_GNSS, NULL, TASK_PRIO_GNSS, NULL);
    ESP_LOGI(TAG, "GNSS init ok: UART1 TX=%d RX=%d, LDO EN=%d, probe 9600/115200/38400, NMEA+UBX",
             PIN_GNSS_TX, PIN_GNSS_RX, PIN_GNSS_LDO_EN);
    return ESP_OK;
}

void gnss_get_data(gnss_data_t *out)
{
    if (out == NULL) {
        return;
    }
    if (s_mutex == NULL) {
        memset(out, 0, sizeof(*out));
        return;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *out = s_data;
        xSemaphoreGive(s_mutex);
    } else {
        memset(out, 0, sizeof(*out));
        return;
    }

    /* 定位数据 5s 未更新则视为失效，避免 UI 显示陈旧坐标 */
    uint64_t now = now_ms();
    if (out->last_fix_ms == 0 || now - out->last_fix_ms > 5000) {
        out->valid = false;
    }
}

void gnss_get_sat_info(gnss_sat_info_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    if (s_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    gnss_sat_info_t tmp = s_sat;
    xSemaphoreGive(s_mutex);

    uint64_t now = now_ms();
    uint8_t n = 0;
    uint8_t used = 0;
    for (uint8_t i = 0; i < tmp.count; i++) {
        if (now - tmp.sats[i].last_ms > GNSS_SAT_STALE_MS) {
            continue;
        }
        out->sats[n] = tmp.sats[i];
        if (tmp.sats[i].used) {
            used++;
        }
        if (tmp.sats[i].sys < GNSS_SYS_COUNT) {
            out->sys_count[tmp.sats[i].sys]++;
        }
        n++;
    }
    out->count = n;
    out->used = used;
    out->gsv_total = tmp.gsv_total;
    out->last_update_ms = tmp.last_update_ms;
    out->last_gsv_ms = tmp.last_gsv_ms;
    out->stale = (tmp.last_gsv_ms == 0) || (now - tmp.last_gsv_ms > GNSS_SAT_NO_GSV_MS);
}

bool gnss_is_fixed(void)
{
    gnss_data_t d;
    gnss_get_data(&d);
    return d.valid && d.fix_type >= 2;
}

void gnss_set_rtc_auto_sync(bool enable)
{
    s_rtc_auto_sync = enable;
}

bool gnss_get_rtc_auto_sync(void)
{
    return s_rtc_auto_sync;
}

void gnss_get_recent_log(char *out_buf, size_t max_len)
{
    if (out_buf == NULL || max_len == 0) {
        return;
    }
    out_buf[0] = '\0';
    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        size_t offset = 0;
        /* 最新一行在最上方，避免终端卡片高度不足时只看到旧报文 */
        for (int i = 0; i < s_gnss_log_count; i++) {
            int idx = (s_gnss_log_head - 1 - i + GNSS_LOG_RING_SIZE * 2) % GNSS_LOG_RING_SIZE;
            int written = snprintf(out_buf + offset, max_len - offset, "%s\n", s_gnss_log_ring[idx]);
            if (written < 0 || (size_t)written >= max_len - offset) {
                break;
            }
            offset += (size_t)written;
        }
        xSemaphoreGive(s_mutex);
    }
}

void gnss_set_update_rate(uint8_t hz)
{
    if (hz != 1 && hz != 5 && hz != 10 && hz != 18) {
        hz = 10;
    }
    s_req_rate_hz = hz;
    s_req_rate_dirty = true;
}

void gnss_set_gnss_mode(uint8_t mode)
{
    if (mode > GNSS_MODE_GPS_ONLY) {
        mode = GNSS_MODE_GPS_BDS;
    }
    s_req_mode = mode;
    s_req_mode_dirty = true;
}

uint8_t gnss_get_update_rate(void)
{
    return s_effective_rate;
}

uint8_t gnss_get_gnss_mode(void)
{
    return s_effective_mode;
}
