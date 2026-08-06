/*
 * gnss.c - GNSS 驱动实现（NEO-M8N，NMEA + UBX NAV-PVT 双协议解析）
 * 流程：
 *   1. GPIO14 拉高使能 LDO → 模块上电
 *   2. UART1 @9600 启动，事件队列收字节
 *   3. 逐字节组帧：'$' 起 NMEA 行 / 0xB5 0x62 起 UBX 帧
 *   4. 解析 RMC/GGA/GSA（NMEA）或 NAV-PVT（UBX）→ 数据快照
 *   5. 波特率探测：2s 无有效帧 → 切下一档 [9600→38400→115200]，循环
 *   6. RMC/NAV-PVT 时间 → settimeofday（UTC），UI 显示 UTC+8
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

static const uint32_t GNSS_BAUDS[] = { 9600, 38400, 115200 };
#define GNSS_BAUD_CNT       (sizeof(GNSS_BAUDS) / sizeof(GNSS_BAUDS[0]))
#define GNSS_BAUD_HOLD_MS   2000            /* 每档等待时间 */
#define GNSS_LINE_MAX       128
#define GNSS_NMEA_LOG       1               /* 临时调试：打印 NMEA 原始行（每 GNSS_NMEA_LOG_EVERY 行一条，定位后移除） */
#define GNSS_NMEA_LOG_EVERY 10              /* 每 10 条 NMEA 行打印 1 条（NMEA 多句/秒，限频防刷屏） */

static SemaphoreHandle_t s_mutex = NULL;
static gnss_data_t s_data = { 0 };

/* ---- UART 事件队列 ---- */
static QueueHandle_t s_uart_queue = NULL;

/* ---- 帧解析状态 ---- */
static uint8_t s_line[GNSS_LINE_MAX];
static size_t s_line_len = 0;
static bool s_in_ubx = false;
static uint8_t s_ubx_class, s_ubx_id;
static uint16_t s_ubx_len, s_ubx_pos;
static uint8_t s_ubx_ck_a, s_ubx_ck_b;
static uint8_t s_ubx_payload[512];
static uint64_t s_last_frame_ms = 0;        /* 最近有效帧时间 */

static void parse_nmea_line(const char *line, size_t len);
static void parse_ubx_frame(void);
static void set_rtc_time(uint16_t year, uint8_t mon, uint8_t day,
                         uint8_t hour, uint8_t min, uint8_t sec);

/* ==================== NMEA 字段工具 ==================== */
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

/* ==================== 3s 速度滑动平均 ==================== */
#define SPD_AVG_WIN_MS   3000
#define SPD_AVG_SLOTS    32
static uint32_t s_spd_t[SPD_AVG_SLOTS];
static float    s_spd_v[SPD_AVG_SLOTS];
static uint8_t  s_spd_cnt = 0;
static uint8_t  s_spd_idx = 0;

static void speed_avg_push(float v)
{
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
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

/* ==================== NMEA 解析 ==================== */
static void parse_nmea_line(const char *line, size_t len)
{
    (void)len;
#if GNSS_NMEA_LOG
    /* 临时调试：限频打印原始 NMEA 行（验证波特率探测/模块输出） */
    static uint16_t s_nmea_cnt = 0;
    if (++s_nmea_cnt % GNSS_NMEA_LOG_EVERY == 1) {
        char dbg[96];
        size_t n = strlen(line);
        if (n > sizeof(dbg) - 1) {
            n = sizeof(dbg) - 1;
        }
        memcpy(dbg, line, n);
        dbg[n] = '\0';
        ESP_LOGI(TAG, "NMEA: %s", dbg);
    }
#endif
    if (strncmp(line, "$GNRMC", 6) == 0 || strncmp(line, "$GPRMC", 6) == 0) {
        char f[16];
        /* 状态：先判断有效性再同步时间（无效帧不得覆盖 RTC） */
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
        /* 速度（节 → km/h） */
        char v[16] = { 0 };
        double spd = 0;
        if (nmea_field(line, 7, v, sizeof(v)) > 0) {
            spd = atof(v) * 1.852;
        }
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
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
                    s_data.fix_type = 1;    /* 至少 2D 估算 */
                }
            } else {
                s_data.speed_kmh = (float)spd;
            }
            s_data.valid = (s_data.fix_type >= 2) || valid;
            xSemaphoreGive(s_mutex);
        }
        s_last_frame_ms = esp_timer_get_time() / 1000;
        return;
    }
    if (strncmp(line, "$GNGGA", 6) == 0 || strncmp(line, "$GPGGA", 6) == 0) {
        char f[16];
        char latf[16] = { 0 }, lonf[16] = { 0 };
        char ns[4] = { 0 }, ew[4] = { 0 };
        char fix[8] = { 0 }, sats[8] = { 0 }, altf[16] = { 0 }, hdopf[16] = { 0 };
        nmea_field(line, 2, latf, sizeof(latf));
        nmea_field(line, 3, ns, sizeof(ns));
        nmea_field(line, 4, lonf, sizeof(lonf));
        nmea_field(line, 5, ew, sizeof(ew));
        nmea_field(line, 6, fix, sizeof(fix));
        nmea_field(line, 7, sats, sizeof(sats));
        nmea_field(line, 9, altf, sizeof(altf));
        nmea_field(line, 8, hdopf, sizeof(hdopf));
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            int ftype = atoi(fix);
            if (strlen(latf) > 0 && strlen(lonf) > 0 && ftype > 0) {
                s_data.lat = nmea_latlon(atof(latf), ns[0]);
                s_data.lon = nmea_latlon(atof(lonf), ew[0]);
            }
            s_data.fix_type = (uint8_t)ftype;
            s_data.sats = (uint8_t)atoi(sats);
            s_data.alt_m = (float)atof(altf);
            s_data.hdop = (float)atof(hdopf);
            s_data.valid = (ftype > 0);
            xSemaphoreGive(s_mutex);
        }
        s_last_frame_ms = esp_timer_get_time() / 1000;
        return;
    }
    if (strncmp(line, "$GNGSA", 6) == 0 || strncmp(line, "$GPGSA", 6) == 0) {
        char pd[16] = { 0 }, hd[16] = { 0 }, vd[16] = { 0 };
        nmea_field(line, 15, pd, sizeof(pd));
        nmea_field(line, 16, hd, sizeof(hd));
        nmea_field(line, 17, vd, sizeof(vd));
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            s_data.pdop = (float)atof(pd);
            s_data.hdop = (float)atof(hd);
            s_data.vdop = (float)atof(vd);
            xSemaphoreGive(s_mutex);
        }
        s_last_frame_ms = esp_timer_get_time() / 1000;
        return;
    }
    /* GSV：跟踪中的卫星总数（字段 3，每条 GSV 报相同值） */
    if (strncmp(line, "$GPGSV", 6) == 0 || strncmp(line, "$GLGSV", 6) == 0 ||
        strncmp(line, "$GBGSV", 6) == 0 || strncmp(line, "$GAGSV", 6) == 0) {
        char f[8] = { 0 };
        nmea_field(line, 3, f, sizeof(f));
        int tracked = atoi(f);
        if (tracked > 0 && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            s_data.sats_tracked = (uint8_t)tracked;
            xSemaphoreGive(s_mutex);
        }
        s_last_frame_ms = esp_timer_get_time() / 1000;
    }
}

/* ==================== UBX NAV-PVT 解析（M8N 支持） ==================== */
static void parse_ubx_frame(void)
{
    if (s_ubx_class != 0x01 || s_ubx_id != 0x07) {
        return;    /* 仅 NAV-PVT */
    }
    const uint8_t *p = s_ubx_payload;
    if (s_ubx_len < 92) {
        return;
    }
    uint16_t year = (uint16_t)(p[4] | (p[5] << 8));
    uint8_t mon = p[6], day = p[7], hour = p[8], min = p[9], sec = p[10];
    uint8_t fix_type = p[20];
    uint8_t num_sv = p[23];
    int32_t lon = (int32_t)(p[24] | (p[25] << 8) | ((uint32_t)p[26] << 16) | ((uint32_t)p[27] << 24));
    int32_t lat = (int32_t)(p[28] | (p[29] << 8) | ((uint32_t)p[30] << 16) | ((uint32_t)p[31] << 24));
    int32_t height = (int32_t)(p[32] | (p[33] << 8) | ((uint32_t)p[34] << 16) | ((uint32_t)p[35] << 24));
    int32_t vel_n = (int32_t)(p[56] | (p[57] << 8) | ((uint32_t)p[58] << 16) | ((uint32_t)p[59] << 24));
    int32_t vel_e = (int32_t)(p[60] | (p[61] << 8) | ((uint32_t)p[62] << 16) | ((uint32_t)p[63] << 24));
    int32_t g_speed = (int32_t)(p[68] | (p[69] << 8) | ((uint32_t)p[70] << 16) | ((uint32_t)p[71] << 24));
    int32_t head_mot = (int32_t)(p[72] | (p[73] << 8) | ((uint32_t)p[74] << 16) | ((uint32_t)p[75] << 24));
    uint16_t p_dop = (uint16_t)(p[76] | (p[77] << 8));

    if (year >= 2000 && year <= 2099 && fix_type >= 2) {
        set_rtc_time(year, mon, day, hour, min, sec);
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_data.fix_type = fix_type;
        s_data.sats = num_sv;
        s_data.lat = lat / 1e7;
        s_data.lon = lon / 1e7;
        s_data.alt_m = height / 1000.0f;
        s_data.speed_kmh = g_speed * 0.0036f;      /* mm/s → km/h */
        speed_avg_push(s_data.speed_kmh);
        s_data.course_deg = head_mot / 1e5f;
        s_data.pdop = p_dop / 100.0f;
        s_data.hdop = p_dop / 100.0f;               /* M8N NAV-PVT 无独立 HDOP，取 PDOP 近似 */
        s_data.valid = (fix_type >= 2);
        (void)vel_n;
        (void)vel_e;
        xSemaphoreGive(s_mutex);
    }
    s_last_frame_ms = esp_timer_get_time() / 1000;
}

/* ==================== RTC 同步（UTC） ==================== */
static void set_rtc_time(uint16_t year, uint8_t mon, uint8_t day,
                         uint8_t hour, uint8_t min, uint8_t sec)
{
    if (year < 2000 || year > 2099 || mon < 1 || mon > 12 || day < 1 || day > 31) {
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
    struct timeval tv = { .tv_sec = t, .tv_usec = 0 };
    settimeofday(&tv, NULL);
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_data.utc_sec = (uint32_t)t;
        s_data.time_valid = true;
        xSemaphoreGive(s_mutex);
    }
}

/* ==================== 字节流组帧 ==================== */
static void feed_byte(uint8_t b)
{
    if (s_in_ubx) {
        /* UBX 帧：sync(2) class id len(2) payload(len) CK_A CK_B */
        if (s_ubx_pos < 4) {
            /* class, id, len_lo, len_hi（CK 从 class 起累加） */
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
                    s_in_ubx = false;    /* 长度异常，放弃 */
                }
            }
        } else if (s_ubx_pos < 4 + s_ubx_len) {
            s_ubx_payload[s_ubx_pos - 4] = b;
            s_ubx_ck_a += b;
            s_ubx_ck_b += s_ubx_ck_a;
        } else if (s_ubx_pos == 4 + s_ubx_len) {
            if (b != s_ubx_ck_a) {
                s_in_ubx = false;    /* CK_A 不符，丢弃 */
                return;
            }
        } else {
            if (b == s_ubx_ck_b) {
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
            parse_nmea_line((const char *)s_line, s_line_len);
            s_line_len = 0;
        } else if (b == 0xB5) {
            /* UBX 帧紧随 NMEA？重新进 UBX */
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

/* ==================== UBX 发送（波特率升级用） ==================== */
static void ubx_send(uint8_t class_id, uint8_t msg_id, const uint8_t *payload, uint16_t len)
{
    uint8_t buf[128];
    if (len + 8 > sizeof(buf)) {
        return;
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
    uart_write_bytes(UART_NUM_1, buf, len + 8);
}

/* CFG-PRT：UART1 波特率切 115200（8N1，outProto=NMEA+UBX） */
static void ubx_cfg_prt_115200(void)
{
    const uint8_t payload[20] = {
        1,            /* portID: UART1 */
        0,            /* reserved */
        0, 0,         /* txReady */
        0x08, 0xD0, 0x00, 0x00,   /* mode: 0x0000D008 = 115200, 8N1 */
        0, 0, 0, 0,   /* reserved */
        0x01, 0x00,   /* inProtoMask: UBX */
        0x03, 0x00,   /* outProtoMask: NMEA + UBX */
        0, 0,         /* flags */
        0, 0,         /* reserved */
    };
    ubx_send(0x06, 0x00, payload, sizeof(payload));
    ESP_LOGI(TAG, "UBX CFG-PRT sent: switch to 115200");
}

/* ==================== 主任务：收字节 + 波特率探测 ==================== */
static void gnss_task(void *arg)
{
    (void)arg;
    uint32_t baud_idx = 0;
    bool upgraded = false;      /* 已尝试升级 115200 */
    bool upgrade_failed = false;
    uart_set_baudrate(UART_NUM_1, GNSS_BAUDS[baud_idx]);
    ESP_LOGI(TAG, "baud probe start @%lu", (unsigned long)GNSS_BAUDS[baud_idx]);

    uint8_t buf[128];
    for (;;) {
        uint64_t now = esp_timer_get_time() / 1000;
        static uint64_t s_probe_start = 0;

        if (s_last_frame_ms == 0) {
            /* 波特率探测：2s 无有效帧 → 切下一档 */
            if (s_probe_start == 0) {
                s_probe_start = now;
            }
            if (now - s_probe_start > GNSS_BAUD_HOLD_MS) {
                s_probe_start = now;
                baud_idx = (baud_idx + 1) % GNSS_BAUD_CNT;
                uart_set_baudrate(UART_NUM_1, GNSS_BAUDS[baud_idx]);
                ESP_LOGI(TAG, "no data, try @%lu", (unsigned long)GNSS_BAUDS[baud_idx]);
            }
        } else if (!upgraded && !upgrade_failed) {
            /* 已锁定（默认 9600）→ 发 UBX 配置切 115200，尝试更高波特率 */
            ubx_cfg_prt_115200();
            vTaskDelay(pdMS_TO_TICKS(50));
            uart_set_baudrate(UART_NUM_1, 115200);
            upgraded = true;
            ESP_LOGI(TAG, "switched to 115200, waiting data...");
        } else if (upgraded && !upgrade_failed && now - s_last_frame_ms > GNSS_BAUD_HOLD_MS * 2) {
            /* 115200 下 4s 无数据 → 回退 9600（模块未响应配置，仅一次） */
            uart_set_baudrate(UART_NUM_1, 9600);
            upgrade_failed = true;
            ESP_LOGW(TAG, "115200 no data, fallback to 9600");
        } else if (now - s_last_frame_ms > 10000) {
            /* 完全失联 10s → 重新探测（兜底） */
            ESP_LOGW(TAG, "GNSS 失联，重新探测波特率");
            s_last_frame_ms = 0;
            upgraded = false;
            upgrade_failed = false;
            baud_idx = 0;
            s_probe_start = 0;
            uart_set_baudrate(UART_NUM_1, 9600);
        }

        /* 收数据（带超时，避免阻塞探测） */
        int len = uart_read_bytes(UART_NUM_1, buf, sizeof(buf), pdMS_TO_TICKS(100));
        for (int i = 0; i < len; i++) {
            feed_byte(buf[i]);
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

    /* UART1：TX17/RX18 */
    uart_config_t uart_cfg = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(UART_NUM_1, 2048, 0, 20, &s_uart_queue, 0),
                        TAG, "uart_driver_install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(UART_NUM_1, &uart_cfg), TAG, "uart_param_config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(UART_NUM_1, PIN_GNSS_TX, PIN_GNSS_RX, -1, -1),
                        TAG, "uart_set_pin failed");

    xTaskCreate(gnss_task, "gnss_task", TASK_STACK_GNSS, NULL, TASK_PRIO_GNSS, NULL);
    ESP_LOGI(TAG, "GNSS init ok (UART1 17/18, EN=14)");
    return ESP_OK;
}

void gnss_get_data(gnss_data_t *out)
{
    if (out == NULL) {
        return;
    }
    if (s_mutex == NULL) {          /* 未初始化防御（UI 任务可能先于 init 运行） */
        memset(out, 0, sizeof(*out));
        return;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *out = s_data;
        xSemaphoreGive(s_mutex);
    } else {
        memset(out, 0, sizeof(*out));
    }
}

bool gnss_is_fixed(void)
{
    gnss_data_t d;
    gnss_get_data(&d);
    return d.valid && d.fix_type >= 2;
}
