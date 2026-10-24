/*
 * gnss.c - GNSS 驱动实现（NEO-M8N，NMEA + UBX 双协议解析与配置）
 *
 * 启动流程：
 *   1. GPIO14 拉高使能 GNSS LDO → 等待模块启动
 *   2. UART1 @9600 启动；若模块已被旧固件保存为 115200，则回退到 115200 锁定
 *   3. 锁定后通过 UBX-CFG-PRT 升到 115200（必须收到 ACK 才切换）
 *   4. 非阻塞配置状态机：
 *        CFG-PRT → MON-VER → CFG-RATE → CFG-MSG → CFG-GNSS → CFG-NAV5
 *      每条命令等待 ACK/NAK ≤500ms，失败重试 1 次后降级；
 *      **不保存配置**（不发 CFG-CFG save），每次上电回 9600 默认后重新下发；
 *      仅当开机就在 115200（旧固件保存过）时先 CFG-CFG clear 一次。
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
static const uint32_t GNSS_BAUDS[] = { 9600, 115200 };
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

/* 完整 GNSS 自检（UART 环回 / NMEA TX 效果 / PUBX,41 / 多波特率扫描）
 * 默认关闭：正常启动直接 9600 -> CFG-PRT -> 115200 -> 配置，约 1s。
 * 仅在排障时置 1，或链路异常时手动启用。 */
#define GNSS_SELFTEST_FULL          0

/* ==================== 共享状态 ==================== */
static SemaphoreHandle_t s_mutex = NULL;
static SemaphoreHandle_t s_log_mutex = NULL;
static gnss_data_t s_data = { 0 };
static gnss_sat_info_t s_sat = { 0 };
static uint8_t s_gsv_total[GNSS_SYS_COUNT] = { 0 };
static uint32_t s_last_navsat_ms = 0;   /* 最近 NAV-SAT 帧时间；NAV-SAT 为 used 权威源 */
static uint32_t s_last_navpvt_ms = 0;   /* 最近 NAV-PVT 帧时间；NAV-PVT 为 fix/valid 权威源 */

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
static volatile uint16_t s_cnt_ubx_sync = 0;   /* RX 侧看到的 UBX sync（B5 62）次数 */
static volatile uint16_t s_cnt_ubx_frames = 0; /* RX 侧成功解析的 UBX 帧数 */
static uint64_t s_last_sat_log_ms = 0;
static uint64_t s_last_nmea_log_ms = 0;
static uint16_t s_log_writes = 0;
static uint16_t s_log_drops = 0;
static uint8_t s_seen_sys_mask = 0;

/* 一次性 GNSS 自检结果（9600/115200 + PUBX,41 两种协议掩码） */
static bool s_selftest_done = false;
static bool s_st_ubx_9600 = false;
static bool s_st_pubx_9600 = false;
static bool s_st_force_9600 = false;
static bool s_st_ubx_115200 = false;
static bool s_st_pubx_115200 = false;
static bool s_st_nmea_115200 = false;

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

/* NMEA $PUBX,41：设置 UART1 输入/输出协议掩码与波特率。
 * 用于模块 UBX 输入被旧配置关闭、CFG-PRT 无 ACK 的场景。
 * 手册 UBX-13003221 R17 p136：inProto/outProto 位 0=UBX, 1=NMEA, 2=RTCM2, 3=RTCM3。 */
static void nmea_send_pubx41_cfg(uint32_t baud, uint16_t in_proto, uint16_t out_proto)
{
    char body[72];
    int body_len = snprintf(body, sizeof(body), "PUBX,41,1,%04X,%04X,%lu,0",
                            (unsigned)in_proto, (unsigned)out_proto, (unsigned long)baud);
    if (body_len <= 0 || body_len >= (int)sizeof(body)) {
        return;
    }
    uint8_t ck = 0;
    for (int i = 0; i < body_len; i++) {
        ck ^= (uint8_t)body[i];
    }
    char buf[96];
    int n = snprintf(buf, sizeof(buf), "$%s*%02X\r\n", body, ck);
    if (n > 0) {
        uart_write_bytes(UART_NUM_1, buf, (size_t)n);
        ESP_LOGI(TAG, "PUBX,41: %s", buf);
    }
}

/* 兼容旧调用：启用 UBX+NMEA 并切 115200 */
static void nmea_send_pubx41(void)
{
    nmea_send_pubx41_cfg(115200, 0x0003, 0x0003);
}

/* ==================== 配置状态机 ==================== */
typedef enum {
    CFG_STEP_NONE = 0,
    CFG_STEP_UBX_PROBE,
    CFG_STEP_PUBX,
    CFG_STEP_PRT,
    CFG_STEP_MONVER,
    CFG_STEP_GNSSPOLL,
    CFG_STEP_RATE,
    CFG_STEP_MSG,
    CFG_STEP_GNSS,
    CFG_STEP_NAV5,
    CFG_STEP_CLEAR,     /* 清除旧保存配置（只清 BBR/Flash，不保存新配置） */
    CFG_STEP_CLEAR_VERIFY,  /* clear 后确认模块当前波特率（9600 或 115200） */
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

/* ---- CFG-GNSS 回读模板 ----
 * 模块回读的 block 结构（含 resTrkCh/maxTrkCh/信号位）是唯一被接收的组合，
 * 下发时照抄模板、只改写 flags.bit0（enable）。实测（2026-10-23）：
 *   · 只发 GPS+GLO 2 块 -> ACK 但回读不变（被规范化），等于没生效
 *   · 5 块全列但改 enable 位 -> 生效（SBAS/QZSS 可被关闭）
 *   · 任何含 BDS 的请求 -> NAK（OTP 未授权）
 */
#define GNSS_TPL_MAX_BLOCKS  8
static bool s_gnss_tpl_valid = false;
static uint8_t s_gnss_tpl_trk_hw = 32;
static uint8_t s_gnss_tpl_trk_use = 32;
static uint8_t s_gnss_tpl_n = 0;
static gnss_cfg_block_t s_gnss_tpl[GNSS_TPL_MAX_BLOCKS];

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
static bool s_bds_blocked = false;   /* 本模块 NAK 过 BDS -> 永久不再尝试（OTP 未授权）*/
static bool s_cfg_degraded = false;
static bool s_cfg_baud_switched = false;
static bool s_clear_try_115200 = false;   /* clear 后 9600 验证失败 -> 再试 115200 */
static bool s_pubx_tried = false;
static bool s_pubx_used = false;
static bool s_pubx_nmea_seen = false;
static uint8_t s_effective_rate = 10;
static uint8_t s_effective_mode = GNSS_MODE_GPS_GLO_BDS;

/* 请求标志（UI 任务写入，gnss_task 消费） */
static volatile uint8_t s_req_rate_hz = 10;
static volatile uint8_t s_req_mode = GNSS_MODE_GPS_GLO_BDS;
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

/* M8N 更新率硬约束（u-blox 8 并发星座降额）：
 *   单 GPS                -> 最大 10 Hz
 *   GPS+GLONASS / GPS+BDS -> 最大 5 Hz
 * 18 Hz 本模块不支持（UI 选项保留兼容，下发时按 10 Hz 处理）。 */
#define GNSS_RATE_MAX_SINGLE_HZ   10
#define GNSS_RATE_MAX_MULTI_HZ     5

/* 目标模式 -> 更新率上限 */
static uint8_t gnss_rate_limit(uint8_t mode)
{
    return (mode == GNSS_MODE_GPS_ONLY) ? GNSS_RATE_MAX_SINGLE_HZ : GNSS_RATE_MAX_MULTI_HZ;
}

/* 把请求速率压到当前模式允许的上限 */
static uint8_t gnss_rate_clamp(uint8_t hz, uint8_t mode)
{
    uint8_t lim = gnss_rate_limit(mode);
    return (hz > lim) ? lim : hz;
}

static const char *gnss_mode_name(uint8_t mode)
{
    switch (mode) {
    case GNSS_MODE_GPS_GLO_BDS: return "GPS+GLO+BDS";
    case GNSS_MODE_GPS_GLO:     return "GPS+GLONASS";
    case GNSS_MODE_GPS_ONLY:    return "GPS only";
    default:                    return "unknown";
    }
}

/* 目标模式 -> 某个 gnssId 是否应使能（BDS 是否真能开由模块 ACK/NAK 决定） */
static bool gnss_want_sys(uint8_t mode, uint8_t id)
{
    switch (mode) {
    case GNSS_MODE_GPS_ONLY:
        return id == GNSS_SYS_GPS;
    case GNSS_MODE_GPS_GLO:
        return id == GNSS_SYS_GPS || id == GNSS_SYS_GLONASS;
    case GNSS_MODE_GPS_GLO_BDS:
    default:
        /* BDS 被 NAK 过后就不再请求（这颗 NEO-M8N-0 的 OTP 只授权 GPS+GLO） */
        if (id == GNSS_SYS_BEIDOU && s_bds_blocked) {
            return false;
        }
        return id == GNSS_SYS_GPS || id == GNSS_SYS_GLONASS || id == GNSS_SYS_BEIDOU;
    }
}

static const char *cfg_step_name(cfg_step_t step){
    switch (step) {
    case CFG_STEP_UBX_PROBE: return "UBX-PROBE";
    case CFG_STEP_PUBX:      return "PUBX,41";
    case CFG_STEP_PRT:    return "CFG-PRT";
    case CFG_STEP_MONVER: return "MON-VER";
    case CFG_STEP_GNSSPOLL: return "CFG-GNSS-POLL";
    case CFG_STEP_RATE:   return "CFG-RATE";
    case CFG_STEP_MSG:    return "CFG-MSG";
    case CFG_STEP_GNSS:   return "CFG-GNSS";
    case CFG_STEP_NAV5:   return "CFG-NAV5";
    case CFG_STEP_CLEAR:  return "CFG-CFG-CLEAR";
    case CFG_STEP_CLEAR_VERIFY: return "CFG-CFG-VERIFY";
    case CFG_STEP_DONE:   return "DONE";
    case CFG_STEP_FALLBACK: return "FALLBACK";
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
        s_sat.sats[idx].used = false;   /* 新条目必须清零，避免复用旧槽位的 used */
    }
    s_sat.sats[idx].elev = elev;
    s_sat.sats[idx].azim = azim;
    s_sat.sats[idx].cno = cno;
    s_sat.sats[idx].last_ms = (uint32_t)now_ms();
    if (set_used) {
        /* 同一卫星可能有多个信号块：OR 合并，避免后一个未用信号把前一个 used 覆盖掉 */
        s_sat.sats[idx].used = s_sat.sats[idx].used || used;
    }
}

static void sat_remove_sys(uint8_t sys)
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < s_sat.count; i++) {
        /* 保留 used 条目：GSV 每轮清表时不能把 NAV-SAT/GSA 刚置好的 used 标志删掉，
         * 否则 USED 计数会周期性掉到 0 或偏低。 */
        if (s_sat.sats[i].sys != sys || s_sat.sats[i].used) {
            s_sat.sats[n++] = s_sat.sats[i];
        }
    }
    s_sat.count = n;
    if (sys < GNSS_SYS_COUNT) {
        s_gsv_total[sys] = 0;
    }
}

/* NAV-SAT 是否新鲜（2s 内收到过）；新鲜时它是对 used 的唯一权威源 */
static bool navsat_fresh(void)
{
    return s_last_navsat_ms != 0 && (now_ms() - s_last_navsat_ms) <= 2000;
}

/* NAV-PVT 是否新鲜（2s 内收到过）；新鲜时它是对 fix_type/valid 的权威源 */
static bool navpvt_fresh(void)
{
    return s_last_navpvt_ms != 0 && (now_ms() - s_last_navpvt_ms) <= 2000;
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
    /* 统一 UI 口径：跟踪数=实际列表条目数（不是 GSV 的 numSV 汇总）。
     * 在用数由 gnss_get_data() 在 NAV-SAT 新鲜时用 s_sat.used 覆盖，避免
     * NMEA-only 回退场景下被 GSV 周期覆盖成 0。 */
    s_data.sats_tracked = s_sat.count;
}

/* ==================== 时间同步 ==================== */
/* UTC 年月日时分秒 -> Unix epoch，完全不依赖 TZ/mktime。
 * GNSS 给的是 UTC；系统 RTC 必须存 UTC（GPX/日志也用 UTC），
 * 中国时间仅在 UI 显示时 +8h。 */
static time_t utc_epoch(int year, int mon, int day, int hour, int min, int sec)
{
    /* Howard Hinnant days_from_civil */
    int y = year - (mon <= 2);
    int era = (y >= 0 ? y : y - 399) / 400;
    unsigned yoe = (unsigned)(y - era * 400);
    unsigned doy = (unsigned)((153 * (mon + (mon > 2 ? -3 : 9)) + 2) / 5 + day - 1);
    unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    int64_t days = (int64_t)era * 146097 + (int64_t)doe - 719468;
    return (time_t)(days * 86400 + hour * 3600 + min * 60 + sec);
}

static void set_rtc_time(uint16_t year, uint8_t mon, uint8_t day,
                         uint8_t hour, uint8_t min, uint8_t sec)
{
    if (year < 2000 || year > 2099 || mon < 1 || mon > 12 || day < 1 || day > 31 ||
        hour > 23 || min > 59 || sec > 60) {
        return;
    }
    time_t t = utc_epoch((int)year, (int)mon, (int)day, (int)hour, (int)min, (int)sec);
    if (t <= 0) {
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
    if (s_log_mutex && xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
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
        if (s_log_writes < 0xFFFF) {
            s_log_writes++;
        }
        xSemaphoreGive(s_log_mutex);
    } else {
        if (s_log_drops < 0xFFFF) {
            s_log_drops++;
        }
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
            /* NAV-PVT 新鲜时由它管理 fix_type/valid；RMC 仅在回退时参与 */
            if (!navpvt_fresh()) {
                if (s_data.fix_type < 2) {
                    s_data.fix_type = 2;    /* RMC A = 有效定位（无法区分 2D/3D，保守按 2D） */
                }
                s_data.last_fix_ms = (uint32_t)now_ms();
                if (s_data.ttff_ms == 0) {
                    s_data.ttff_ms = (uint32_t)(now_ms() - s_boot_ms);
                }
                s_data.valid = true;
            }
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

    int quality = atoi(fix);
    bool gga_fix = (quality > 0);
    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (gga_fix && strlen(latf) > 0 && strlen(lonf) > 0) {
            s_data.lat = nmea_latlon(atof(latf), ns[0]);
            s_data.lon = nmea_latlon(atof(lonf), ew[0]);
        }
        /* 注意：GGA field 6 是“定位质量”(0/1/2/4/5/6)，不是 2D/3D；
         * 不能写进 fix_type（旧代码把它当 fixType，导致 UI 在 2D/3D 间跳）。
         * fix_type 由 NAV-PVT 权威给出；无 NAV-PVT 时由 GSA field 2 回退。 */
        s_data.sats = (uint8_t)atoi(sats);
        s_data.alt_m = (float)atof(altf);
        s_data.hdop = (float)atof(hdopf);

        /* NAV-PVT 新鲜时由它统一管理 fix_type/valid，GGA 不再插手 */
        if (!navpvt_fresh()) {
            if (gga_fix) {
                s_data.last_fix_ms = (uint32_t)now_ms();
                if (s_data.ttff_ms == 0) {
                    s_data.ttff_ms = (uint32_t)(now_ms() - s_boot_ms);
                }
            }
            s_data.valid = gga_fix ||
                           (s_data.last_fix_ms != 0 && (uint32_t)now_ms() - s_data.last_fix_ms < 5000);
        }
        xSemaphoreGive(s_mutex);
    }
}

/* NMEA 4.10 GSA field 18: 1=GPS 2=GLONASS 3=Galileo 4=BeiDou 5=QZSS */
static int nmea_gsa_sys_id(const char *line)
{
    char f[8] = { 0 };
    if (nmea_field(line, 18, f, sizeof(f)) <= 0) {
        return -1;
    }
    switch (atoi(f)) {
    case 1: return GNSS_SYS_GPS;
    case 2: return GNSS_SYS_GLONASS;
    case 3: return GNSS_SYS_GALILEO;
    case 4: return GNSS_SYS_BEIDOU;
    case 5: return GNSS_SYS_QZSS;
    default: return -1;
    }
}

static void parse_gsa(const char *line)
{
    int sys = talker_sys(line);
    int sid_sys = nmea_gsa_sys_id(line);
    if (sid_sys >= 0) {
        sys = sid_sys;      /* NMEA 4.10：优先用 field 18 精确到系统，避免 GNGSA 清空所有系统 */
    } else if (line[1] == 'G' && line[2] == 'N') {
        sys = -1;           /* 旧版 GNGSA 无 system ID，按 PRN 匹配所有系统 */
    }
    char pd[16] = { 0 }, hd[16] = { 0 }, vd[16] = { 0 }, ft[8] = { 0 };
    nmea_field(line, 15, pd, sizeof(pd));
    nmea_field(line, 16, hd, sizeof(hd));
    nmea_field(line, 17, vd, sizeof(vd));
    nmea_field(line, 2, ft, sizeof(ft));      /* GSA field 2: 1=no fix, 2=2D, 3=3D */
    int gsa_fix = atoi(ft);

    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_data.pdop = (float)atof(pd);
        s_data.hdop = (float)atof(hd);
        s_data.vdop = (float)atof(vd);

        /* NAV-PVT 不可用时，用 GSA 的 fix type 回退 2D/3D（GGA quality 无法区分） */
        if (!navpvt_fresh() && gsa_fix >= 1 && gsa_fix <= 3) {
            s_data.fix_type = (gsa_fix >= 2) ? (uint8_t)gsa_fix : 0;
        }

        /* GSA 只作为 DOP 来源；used 由 NAV-SAT flags.bit3(svUsed) 独家维护。
         * 仅当 NAV-SAT 不可用（未收到/已超时）时才回退到 GSA 的卫星列表。 */
        if (!navsat_fresh()) {
            sat_clear_used(sys);
            for (int i = 0; i < 12; i++) {
                char f[8] = { 0 };
                if (nmea_field(line, 3 + i, f, sizeof(f)) > 0) {
                    sat_set_used(sys, (uint16_t)atoi(f));
                }
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
            /* u-blox 的 GPGSV 可能同时包含 GPS/SBAS/QZSS（PRN 120+/193+），
             * 清表时一并清掉，避免旧条目残留。 */
            if (sys == GNSS_SYS_GPS) {
                sat_remove_sys(GNSS_SYS_SBAS);
                sat_remove_sys(GNSS_SYS_QZSS);
            }
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
            int prn_val = atoi(prn);
            int sat_sys = sys;
            /* GPGSV 的 PRN 段：1-32 GPS，120-158 SBAS，193-202 QZSS */
            if (sys == GNSS_SYS_GPS) {
                if (prn_val >= 120 && prn_val <= 158) {
                    sat_sys = GNSS_SYS_SBAS;
                } else if (prn_val >= 193 && prn_val <= 202) {
                    sat_sys = GNSS_SYS_QZSS;
                }
            }
            sat_upsert((uint8_t)sat_sys, (uint8_t)prn_val,
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

    /* 首次锁定后打印 8 条原始 NMEA，之后每 10s 采样 1 条，避免刷屏 */
    static uint8_t s_nmea_dbg_left = 8;
    static uint64_t s_nmea_dbg_last_ms = 0;
    uint64_t nmea_now = now_ms();
    if (s_nmea_dbg_left > 0) {
        s_nmea_dbg_left--;
        ESP_LOGI(TAG, "NMEA[%u]: %s", (unsigned)(8 - s_nmea_dbg_left), line);
        if (s_nmea_dbg_left == 0) {
            s_nmea_dbg_last_ms = nmea_now;
        }
    } else if (nmea_now - s_nmea_dbg_last_ms >= 10000) {
        s_nmea_dbg_last_ms = nmea_now;
        ESP_LOGI(TAG, "NMEA sample: %s", line);
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
    uint32_t v_acc = (uint32_t)p[44] | ((uint32_t)p[45] << 8) |
                     ((uint32_t)p[46] << 16) | ((uint32_t)p[47] << 24);
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
        s_last_navpvt_ms = (uint32_t)now_ms();
        s_data.fix_type = fix_type;
        s_data.sats = num_sv;
        s_data.vacc_mm = v_acc;
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
    uint8_t version = p[4];
    if (version != 1) {
        static bool s_bad_ver_logged = false;
        if (!s_bad_ver_logged) {
            s_bad_ver_logged = true;
            ESP_LOGW(TAG, "NAV-SAT version %u unsupported (expect 1), ignore", (unsigned)version);
        }
        return;
    }
    uint8_t num = p[5];
    if (num > GNSS_MAX_SATS) {
        num = GNSS_MAX_SATS;
    }

    if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        sat_recount();      /* 先淘汰 30s 旧条目 */
        sat_clear_used(-1); /* NAV-SAT 是 used 权威源：每帧重建完整 used 集合 */
        for (uint8_t i = 0; i < num; i++) {
            int b = 8 + i * 12;
            if (b + 12 > s_ubx_len) {
                break;
            }
            /* UBX-13003221 R17 p331（protocol 15/18，version 1，12 字节 block）：
             *   +0 gnssId, +1 svId, +2 cno, +3 elev(I1), +4 azim(I2),
             *   +6 prRes(I2, 0.1m), +8 flags(X4)。
             * 协议 15/18 没有 prn 字段；+6 是伪距残差，不能当 PRN。 */
            uint8_t sys = p[b];
            uint8_t sv_id = p[b + 1];
            if (sv_id == 0 || sv_id == 255) {
                continue;   /* 未知卫星 */
            }
            uint8_t cno = p[b + 2];
            int8_t elev_i = (int8_t)p[b + 3];
            uint16_t azim = (uint16_t)(p[b + 4] | (p[b + 5] << 8));
            uint32_t flags = (uint32_t)p[b + 8] | ((uint32_t)p[b + 9] << 8) |
                             ((uint32_t)p[b + 10] << 16) | ((uint32_t)p[b + 11] << 24);

            /* UBX svId -> NMEA/GSV PRN 编号：
             *   GLONASS: slot 1..32 -> PRN 65..96
             *   QZSS:    1..10     -> PRN 193..202
             *   其余（GPS/SBAS/Galileo/BeiDou）svId 即 PRN。 */
            uint16_t prn = sv_id;
            if (sys == GNSS_SYS_GLONASS) {
                prn = (uint16_t)sv_id + 64;
            } else if (sys == GNSS_SYS_QZSS) {
                prn = (uint16_t)sv_id + 192;
            }
            uint8_t elev = (elev_i < 0) ? 0 : (uint8_t)elev_i;
            if (elev > 90) {
                elev = 90;
            }

            /* UBX-NAV-SAT flags 位域：qualityInd:3 (bit0~2)，svUsed:1 (bit3)，...
             * 所以 svUsed = 0x08，不是 bit0。 */
            sat_upsert(sys, (uint8_t)(prn > 255 ? 255 : prn), elev, azim,
                       cno, true, (flags & 0x08) != 0);
        }
        s_last_navsat_ms = (uint32_t)now_ms();
        s_sat.last_gsv_ms = s_last_navsat_ms;
        sat_recount();
        xSemaphoreGive(s_mutex);
    }
}

static void parse_ubx_frame(void)
{
    uint64_t now = now_ms();
    s_cnt_ubx_frames++;

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
        return;
    }
    if (s_ubx_class == 0x06 && s_ubx_id == 0x00 && s_ubx_len >= 20) {   /* CFG-PRT poll 回读 */
        uint32_t mode = (uint32_t)s_ubx_payload[4] | ((uint32_t)s_ubx_payload[5] << 8) |
                        ((uint32_t)s_ubx_payload[6] << 16) | ((uint32_t)s_ubx_payload[7] << 24);
        uint32_t baud = (uint32_t)s_ubx_payload[8] | ((uint32_t)s_ubx_payload[9] << 8) |
                        ((uint32_t)s_ubx_payload[10] << 16) | ((uint32_t)s_ubx_payload[11] << 24);
        uint16_t in_proto = (uint16_t)(s_ubx_payload[12] | (s_ubx_payload[13] << 8));
        uint16_t out_proto = (uint16_t)(s_ubx_payload[14] | (s_ubx_payload[15] << 8));
        ESP_LOGI(TAG, "CFG-PRT READBACK: port=%u mode=0x%08lX baud=%lu in=0x%04X out=0x%04X",
                 (unsigned)s_ubx_payload[0], (unsigned long)mode, (unsigned long)baud,
                 (unsigned)in_proto, (unsigned)out_proto);
        return;
    }
    if (s_ubx_class == 0x06 && s_ubx_id == 0x08 && s_ubx_len >= 6) {   /* CFG-RATE poll 回读 */
        uint16_t meas = (uint16_t)(s_ubx_payload[0] | (s_ubx_payload[1] << 8));
        uint16_t nav = (uint16_t)(s_ubx_payload[2] | (s_ubx_payload[3] << 8));
        uint16_t tref = (uint16_t)(s_ubx_payload[4] | (s_ubx_payload[5] << 8));
        ESP_LOGI(TAG, "CFG-RATE READBACK: meas=%ums (%uHz) nav=%u timeRef=%u",
                 (unsigned)meas, (unsigned)(meas ? 1000 / meas : 0),
                 (unsigned)nav, (unsigned)tref);
        return;
    }
    if (s_ubx_class == 0x06 && s_ubx_id == 0x3E && s_ubx_len >= 4) {   /* CFG-GNSS poll 回读 */
        static const char *names[8] = { "GPS", "SBAS", "GAL", "BDS", "IMES", "QZSS", "GLO", "IRNSS" };
        uint8_t n = s_ubx_payload[3];
        ESP_LOGI(TAG, "CFG-GNSS READBACK: numTrkChHw=%u use=%u blocks=%u",
                 (unsigned)s_ubx_payload[1], (unsigned)s_ubx_payload[2], (unsigned)n);
        for (uint8_t i = 0; i < n; i++) {
            int b = 4 + i * 8;
            if (b + 8 > s_ubx_len) {
                break;
            }
            uint8_t id = s_ubx_payload[b];
            uint32_t flags = (uint32_t)s_ubx_payload[b + 4] |
                             ((uint32_t)s_ubx_payload[b + 5] << 8) |
                             ((uint32_t)s_ubx_payload[b + 6] << 16) |
                             ((uint32_t)s_ubx_payload[b + 7] << 24);
            ESP_LOGI(TAG, "  %s id=%u res=%u max=%u flags=0x%08lX enable=%u sig=0x%lX",
                     id < 8 ? names[id] : "?", (unsigned)id,
                     (unsigned)s_ubx_payload[b + 1], (unsigned)s_ubx_payload[b + 2],
                     (unsigned long)flags, (unsigned)(flags & 0x01),
                     (unsigned long)((flags >> 16) & 0xFF));
        }

        /* 缓存为下发模板：res/max/信号位全部照抄，仅下行时改 enable 位 */
        if (n > 0) {
            uint8_t keep = (n > GNSS_TPL_MAX_BLOCKS) ? GNSS_TPL_MAX_BLOCKS : n;
            s_gnss_tpl_trk_hw = s_ubx_payload[1] ? s_ubx_payload[1] : 32;
            s_gnss_tpl_trk_use = s_ubx_payload[2] ? s_ubx_payload[2] : 32;
            for (uint8_t i = 0; i < keep; i++) {
                int b = 4 + i * 8;
                s_gnss_tpl[i].id = s_ubx_payload[b];
                s_gnss_tpl[i].res = s_ubx_payload[b + 1];
                s_gnss_tpl[i].max = s_ubx_payload[b + 2];
                s_gnss_tpl[i].flags = (uint32_t)s_ubx_payload[b + 4] |
                                      ((uint32_t)s_ubx_payload[b + 5] << 8) |
                                      ((uint32_t)s_ubx_payload[b + 6] << 16) |
                                      ((uint32_t)s_ubx_payload[b + 7] << 24);
            }
            s_gnss_tpl_n = keep;
            s_gnss_tpl_valid = true;
        }
        return;
    }
}

/* ==================== 字节流组帧 ==================== */
static void feed_byte(uint8_t b)
{
    s_last_rx_ms = now_ms();

    if (s_in_ubx) {
        /* UBX 帧：B5 62 | class | id | lenLo | lenHi | payload | CK_A | CK_B
         * pos 语义：0=第二个同步字节(0x62)、1=class、2=id、3=lenLo、4=lenHi，payload 自 5 开始。
         *
         * 【历史 bug（2026-10-23）】旧实现进入 UBX 模式时 pos=0 直接收 class，
         * 把 0x62 当成了 class，于是 len 被算成 0xB8xx（>512）而立即丢弃，
         * 导致模块的所有 UBX 应答（MON-VER/ACK/NAV-PVT…）永远解析不出来，
         * 表现为“发了命令没反应”，同时 u-center 一切正常。 */
        if (s_ubx_pos < 5) {
            if (s_ubx_pos == 0) {
                if (b != 0x62) {
                    s_in_ubx = false;       /* 第二同步字节不符，重新找 B5 */
                    return;
                }
            } else if (s_ubx_pos == 1) {
                s_ubx_class = b;
                s_ubx_ck_a = s_ubx_class;
                s_ubx_ck_b = s_ubx_ck_a;
            } else if (s_ubx_pos == 2) {
                s_ubx_id = b;
                s_ubx_ck_a += s_ubx_id;
                s_ubx_ck_b += s_ubx_ck_a;
            } else if (s_ubx_pos == 3) {
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
        } else if (s_ubx_pos < 5 + s_ubx_len) {
            s_ubx_payload[s_ubx_pos - 5] = b;
            s_ubx_ck_a += b;
            s_ubx_ck_b += s_ubx_ck_a;
        } else if (s_ubx_pos == 5 + s_ubx_len) {
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
        s_cnt_ubx_sync++;
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
            if (s_data.baud == 115200 && s_line_len >= 4 && s_line[0] == '$') {
                /* 允许波特率切换瞬间丢失开头 $GN 的截断句（如 $TXT,...） */
                if (strstr((const char *)s_line, "GGA") || strstr((const char *)s_line, "RMC") ||
                    strstr((const char *)s_line, "GSA") || strstr((const char *)s_line, "GSV") ||
                    strstr((const char *)s_line, "TXT") || strstr((const char *)s_line, "GLL") ||
                    strstr((const char *)s_line, "VTG") || strstr((const char *)s_line, "ZDA")) {
                    s_pubx_nmea_seen = true;    /* 115200 下收到已知 NMEA，说明 PUBX,41 已生效 */
                }
            }
            parse_nmea_line((const char *)s_line, s_line_len);
            s_line_len = 0;
        } else if (b == 0xB5) {
            s_cnt_ubx_sync++;
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
/* 组装 UBX 帧（含 CK_A/CK_B），返回帧长，0 表示失败
 *
 * 注意：CK_A/CK_B 必须按 class → id → lenLo → lenHi → payload 的**顺序累加**，
 *       CK_B 是 CK_A 的累加和。旧实现把 payload 与 header 分开算，CK_A 恰好对、
 *       CK_B 永远错，导致模块静默丢弃所有 UBX 帧（UBX 配置完全不可用）。 */
static size_t ubx_build(uint8_t class_id, uint8_t msg_id, const uint8_t *payload, uint16_t len,
                        uint8_t *out, size_t out_size)
{
    if (len > 0 && payload == NULL) {
        return 0;
    }
    if ((size_t)len + 8 > out_size) {
        return 0;
    }
    out[0] = 0xB5;
    out[1] = 0x62;
    out[2] = class_id;
    out[3] = msg_id;
    out[4] = (uint8_t)(len & 0xFF);
    out[5] = (uint8_t)(len >> 8);
    for (uint16_t i = 0; i < len; i++) {
        out[6 + i] = payload[i];
    }

    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < (size_t)6 + len; i++) {
        ck_a = (uint8_t)(ck_a + out[i]);
        ck_b = (uint8_t)(ck_b + ck_a);
    }
    out[6 + len] = ck_a;
    out[7 + len] = ck_b;
    return (size_t)len + 8;
}

static bool ubx_send(uint8_t class_id, uint8_t msg_id, const uint8_t *payload, uint16_t len)
{
    uint8_t buf[128];
    size_t n = ubx_build(class_id, msg_id, payload, len, buf, sizeof(buf));
    if (n == 0) {
        return false;
    }
    int written = uart_write_bytes(UART_NUM_1, buf, n);
    return written == (int)n;
}

/* 回归保护 2：UBX 解析器自检——喂入一个合成 ACK 帧，必须被完整解析。
 *
 * 历史 bug：进入 UBX 模式时没吃第二同步字节 0x62，导致所有应答被当成
 * “长度 0xB8xx”而丢弃（MON-VER/ACK/NAV-PVT 全部失效）。
 * 这里直接跑一遍解析路径，并在结束后恢复所有解析状态，不影响真实串口流。
 */
static void ubx_parse_selfcheck(void)
{
    uint8_t f[16];
    static const uint8_t ack_pl[2] = { 0x06, 0x08 };
    size_t n = ubx_build(0x05, 0x01, ack_pl, 2, f, sizeof(f));
    if (n == 0) {
        return;
    }

    /* 快照 -> 喂包 -> 校验 -> 还原（含锁检测用的 s_last_frame_ms/s_last_rx_ms）*/
    bool s_in = s_in_ubx;
    uint16_t s_pos = s_ubx_pos, s_len = s_ubx_len;
    uint8_t s_cls = s_ubx_class, s_id = s_ubx_id, s_a = s_ubx_ck_a, s_b = s_ubx_ck_b;
    size_t s_lilen = s_line_len;
    uint64_t s_lf = s_last_frame_ms, s_lr = s_last_rx_ms;
    uint16_t s_frames = s_stat_frames;
    uint16_t before = s_cnt_ubx_frames;

    s_line_len = 0;
    s_in_ubx = false;
    for (size_t i = 0; i < n; i++) {
        feed_byte(f[i]);
    }
    bool ok = (s_cnt_ubx_frames != before) && (s_ubx_class == 0x05) && (s_ubx_id == 0x01) &&
              (s_ubx_len == 2) && (s_ubx_payload[0] == 0x06) && (s_ubx_payload[1] == 0x08);
    uint8_t obs_cls = s_ubx_class, obs_id = s_ubx_id;
    uint16_t obs_len = s_ubx_len;
    bool obs_parsed = (s_cnt_ubx_frames != before);

    s_in_ubx = s_in;
    s_ubx_pos = s_pos;
    s_ubx_len = s_len;
    s_ubx_class = s_cls;
    s_ubx_id = s_id;
    s_ubx_ck_a = s_a;
    s_ubx_ck_b = s_b;
    s_line_len = s_lilen;
    s_last_frame_ms = s_lf;
    s_last_rx_ms = s_lr;
    s_stat_frames = s_frames;
    s_cnt_ubx_frames = before;      /* 不污染统计计数 */

    if (ok) {
        ESP_LOGI(TAG, "UBX parser self-check ok (B5 62 同步字节处理正确)");
    } else {
        ESP_LOGE(TAG, "UBX parser self-check FAILED: parsed=%d cls=0x%02X id=0x%02X len=%u",
                 (int)obs_parsed, (unsigned)obs_cls, (unsigned)obs_id, (unsigned)obs_len);
    }
}

/* 回归保护：MON-VER poll 的校验和必须恰好是 CK_A=0x0E, CK_B=0x34（手册值），
 * 否则说明帧构造被改坏 —— 历史上这个 bug 让全部 UBX 配置静默失效。 */
static void ubx_ck_selfcheck(void)
{
    uint8_t buf[8];
    size_t n = ubx_build(0x0A, 0x04, NULL, 0, buf, sizeof(buf));
    if (n != 8 || buf[6] != 0x0E || buf[7] != 0x34) {
        ESP_LOGE(TAG, "UBX checksum self-check FAILED: got %02X %02X, expect 0E 34",
                 (unsigned)(n == 8 ? buf[6] : 0), (unsigned)(n == 8 ? buf[7] : 0));
    } else {
        ESP_LOGI(TAG, "UBX checksum self-check ok (MON-VER poll CK=0E 34)");
    }
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
    /* 星座确定后再校一次速率上限（例如 BDS 被 NAK 降为 GPS+GLO 后，10 Hz 必须降到 5 Hz）*/
    uint8_t lim = gnss_rate_limit(s_effective_mode);
    if (s_effective_rate > lim) {
        ESP_LOGW(TAG, "rate %uHz 超出 %s 上限，降为 %uHz 并重新下发",
                 (unsigned)s_effective_rate, gnss_mode_name(s_effective_mode), (unsigned)lim);
        s_effective_rate = lim;
        s_msg_idx = 0;
        cfg_goto(CFG_STEP_RATE, 0, now);
        return;
    }
    s_cfg_step = CFG_STEP_DONE;
    s_data.rate_hz = s_effective_rate;
    s_data.ubx_ok = true;
    s_data.cfg_state = s_cfg_degraded ? GNSS_CFG_DEGRADED : GNSS_CFG_OK;
    ESP_LOGI(TAG, "UBX config %s: rate=%uHz mode=%s baud=%lu (RAM only, not saved)",
             s_cfg_degraded ? "DEGRADED" : "OK",
             (unsigned)s_effective_rate, gnss_mode_name(s_effective_mode),
             (unsigned long)s_data.baud);
    /* 回读验证：确认 CFG-PRT / CFG-RATE / CFG-GNSS 真正写入模块 */
    uint8_t port_id = 0x01;
    ubx_send(0x06, 0x00, &port_id, 1);
    ubx_send(0x06, 0x08, NULL, 0);
    ubx_send(0x06, 0x3E, NULL, 0);
}

static void cfg_fallback(uint64_t now)
{
    (void)now;
    s_cfg_step = CFG_STEP_FALLBACK;
    s_data.ubx_ok = false;
    s_data.rate_hz = 1;     /* NMEA-only 默认 1Hz */
    s_data.cfg_state = GNSS_CFG_FALLBACK;
    ESP_LOGW(TAG, "UBX unavailable, fallback to NMEA-only @%lu", (unsigned long)s_data.baud);
}

static void cfg_send_current(uint64_t now)
{
    switch (s_cfg_step) {
    case CFG_STEP_UBX_PROBE:
        ESP_LOGI(TAG, "UBX probe: poll MON-VER @%lu", (unsigned long)s_data.baud);
        s_monver_ms = 0;
        ubx_send(0x0A, 0x04, NULL, 0);
        s_cfg_sent_ms = now;
        s_cfg_waiting_ack = false;
        break;
    case CFG_STEP_PUBX:
        nmea_send_pubx41();
        s_cfg_sent_ms = now;
        s_cfg_next_ms = now + 300;
        s_cfg_waiting_ack = false;
        break;
    case CFG_STEP_PRT: {
        /* portID, reserved, txReady, mode=0x000008D0(8N1；bit4 reserved=1，
         * 手册/ublox_msgs 的规范值), baud=0x0001C200,
         * inProto=UBX+NMEA+RTCM2(0x0007), outProto=NMEA+UBX */
        const uint8_t p[20] = {
            0x01, 0x00,
            0x00, 0x00,
            0xD0, 0x08, 0x00, 0x00,
            0x00, 0xC2, 0x01, 0x00,
            0x07, 0x00,
            0x03, 0x00,
            0x00, 0x00,
            0x00, 0x00,
        };
        ESP_LOGI(TAG, "CFG-PRT -> UART1 115200 8N1 mode=0x8D0 in=0x0007 out=0x0003 (wait ACK)");
        cfg_send_cmd(0x06, 0x00, p, sizeof(p), now);
        break;
    }
    case CFG_STEP_GNSSPOLL:
        /* 先回读星座配置作为下发模板：模块只接受“回读结构 + 改 enable”这种写法 */
        ESP_LOGI(TAG, "poll CFG-GNSS (readback as template)");
        s_gnss_tpl_valid = false;
        s_cfg_sent_ms = now;
        s_cfg_waiting_ack = false;
        ubx_send(0x06, 0x3E, NULL, 0);
        break;
    case CFG_STEP_MONVER:
        ESP_LOGI(TAG, "poll MON-VER (wait response)");
        s_monver_ms = 0;
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
        uint8_t rate = (s_msg_idx <= 1) ? 1 : nmea_rate;   /* NAV-PVT / NAV-SAT 每个解算都发 */
        static const char *msg_name[CFG_MSG_COUNT] = { "NAV-PVT", "NAV-SAT", "GGA", "RMC", "GSA", "GSV" };
        ESP_LOGI(TAG, "CFG-MSG: %s UART1 rate=%u", msg_name[s_msg_idx], (unsigned)rate);
        uint8_t p[8] = { msg_cls[s_msg_idx], msg_id[s_msg_idx], 0, rate, 0, 0, 0, 0 };
        cfg_send_cmd(0x06, 0x01, p, sizeof(p), now);
        break;
    }
    case CFG_STEP_GNSS: {
        /* 这颗模块（PROTVER 15.00）的硬约束：
         *   1) 必须把回读的全部 block 一起下发；只发 2 块会被 ACK 但回读不变（被规范化）
         *   2) 任何含 BDS(gnssId=3) 的 enable 请求 -> NAK（OTP 未授权）
         * 故：以回读模板为骨架，照抄 res/max/信号位，仅改写 flags.bit0 */
        if (!s_gnss_tpl_valid || s_gnss_tpl_n == 0) {
            ESP_LOGW(TAG, "CFG-GNSS skipped: no readback template, keep receiver default");
            s_cfg_degraded = true;
            cfg_goto(CFG_STEP_NAV5, 0, now);
            break;
        }
        uint8_t p[4 + 8 * GNSS_TPL_MAX_BLOCKS] = { 0 };
        p[0] = 0x00;
        p[1] = s_gnss_tpl_trk_hw;
        p[2] = s_gnss_tpl_trk_use;
        p[3] = s_gnss_tpl_n;
        for (uint8_t i = 0; i < s_gnss_tpl_n; i++) {
            uint32_t flags = s_gnss_tpl[i].flags;
            if (gnss_want_sys(s_effective_mode, s_gnss_tpl[i].id)) {
                flags |= 0x00000001u;
            } else {
                flags &= ~0x00000001u;
            }
            p[4 + i * 8 + 0] = s_gnss_tpl[i].id;
            p[4 + i * 8 + 1] = s_gnss_tpl[i].res;
            p[4 + i * 8 + 2] = s_gnss_tpl[i].max;
            p[4 + i * 8 + 3] = 0x00;
            p[4 + i * 8 + 4] = (uint8_t)(flags & 0xFF);
            p[4 + i * 8 + 5] = (uint8_t)((flags >> 8) & 0xFF);
            p[4 + i * 8 + 6] = (uint8_t)((flags >> 16) & 0xFF);
            p[4 + i * 8 + 7] = (uint8_t)((flags >> 24) & 0xFF);
        }
        ESP_LOGI(TAG, "CFG-GNSS request: %s (%u blocks, flags 照回读值仅改 enable)",
                 gnss_mode_name(s_effective_mode), (unsigned)s_gnss_tpl_n);
        cfg_send_cmd(0x06, 0x3E, p, (uint16_t)(4 + 8 * s_gnss_tpl_n), now);
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
    case CFG_STEP_CLEAR: {
        /* 清除 BBR/Flash 里旧的保存配置（含 115200 波特率），保证下次上电回到 9600 默认。
         * 本次会话的 RAM 配置不变，继续在 115200 上完成配置；
         * 不发送 saveMask —— 本固件不再持久化任何新配置。 */
        uint8_t p[13] = { 0 };
        p[0] = 0xDF;        /* clearMask = 0x000003DF（与旧 saveMask 同域） */
        p[1] = 0x03;
        p[12] = 0x03;       /* deviceMask = BBR + Flash */
        ESP_LOGI(TAG, "CFG-CFG clear BBR+Flash (remove legacy saved config, next boot 9600)");
        cfg_send_cmd(0x06, 0x09, p, sizeof(p), now);
        break;
    }
    case CFG_STEP_CLEAR_VERIFY:
        ESP_LOGI(TAG, "CFG-CFG verify link @%lu (poll MON-VER)", (unsigned long)s_data.baud);
        s_monver_ms = 0;
        ubx_send(0x0A, 0x04, NULL, 0);
        s_cfg_sent_ms = now;
        s_cfg_waiting_ack = false;
        break;
    default:
        break;
    }
}

static void cfg_on_success(uint64_t now)
{
    switch (s_cfg_step) {
    case CFG_STEP_UBX_PROBE:
        ESP_LOGI(TAG, "UBX probe OK @%lu", (unsigned long)s_data.baud);
        cfg_goto(CFG_STEP_PRT, 0, now);
        break;
    case CFG_STEP_PRT:
        uart_set_baudrate(UART_NUM_1, 115200);
        uart_flush_input(UART_NUM_1);
        s_data.baud = 115200;
        s_cfg_baud_switched = true;
        ESP_LOGI(TAG, "UBX CFG-PRT ACK: switched to 115200");
        cfg_goto(CFG_STEP_MONVER, 100, now);
        break;
    case CFG_STEP_MONVER:
        s_cfg_baud_switched = false;    /* 已收到 UBX 响应，链路确认成功 */
        if (s_data.baud == 9600) {
            /* 9600 下 UBX 已通，下一步升 115200 */
            cfg_goto(CFG_STEP_PRT, 0, now);
        } else {
            if (s_pubx_used) {
                ESP_LOGI(TAG, "UBX recovered via PUBX,41 @%lu", (unsigned long)s_data.baud);
            }
            cfg_goto(CFG_STEP_GNSSPOLL, 0, now);
        }
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
            ESP_LOGI(TAG, "CFG-MSG done: NAV-PVT=1, NAV-SAT=1, NMEA GGA/RMC/GSA/GSV=%u",
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
        cfg_finish(now);
        break;
    case CFG_STEP_CLEAR:
        ESP_LOGI(TAG, "UBX CFG-CFG clear ACK: legacy config removed (next boot 9600 default)");
        /* clear 可能触发 ioPort 复位：先按 9600 确认，失败再回 115200 */
        s_clear_try_115200 = false;
        uart_set_baudrate(UART_NUM_1, 9600);
        uart_flush_input(UART_NUM_1);
        s_data.baud = 9600;
        s_cfg_baud_switched = false;
        cfg_goto(CFG_STEP_CLEAR_VERIFY, 50, now);
        break;
    case CFG_STEP_CLEAR_VERIFY:
        if (s_data.baud == 9600) {
            ESP_LOGI(TAG, "CFG-CFG verify OK @9600: module reset to default baud, switch to 115200");
            cfg_goto(CFG_STEP_PRT, 0, now);
        } else {
            ESP_LOGI(TAG, "CFG-CFG verify OK @115200: keep current baud");
            cfg_goto(CFG_STEP_GNSSPOLL, 0, now);
        }
        break;
    default:
        break;
    }
}

static void cfg_on_failure(uint64_t now, bool timeout)
{
    ESP_LOGW(TAG, "UBX %s %s", cfg_step_name(s_cfg_step), timeout ? "timeout" : "NAK");

    switch (s_cfg_step) {
    case CFG_STEP_UBX_PROBE:
        if (!s_pubx_tried) {
            s_pubx_tried = true;
            ESP_LOGW(TAG, "UBX probe no response @%lu, try NMEA PUBX,41", (unsigned long)s_data.baud);
            cfg_goto(CFG_STEP_PUBX, 0, now);
        } else {
            cfg_fallback(now);
        }
        break;
    case CFG_STEP_PUBX:
        cfg_fallback(now);
        break;
    case CFG_STEP_PRT:
        if (s_pubx_used) {
            /* PUBX,41 已设置 115200；再试一次 MON-VER，确认 UBX 输出是否可用 */
            ESP_LOGW(TAG, "CFG-PRT no ACK after PUBX,41, try MON-VER");
            cfg_goto(CFG_STEP_MONVER, 0, now);
        } else if (!s_pubx_tried) {
            s_pubx_tried = true;
            ESP_LOGW(TAG, "CFG-PRT no ACK, try NMEA PUBX,41");
            cfg_goto(CFG_STEP_PUBX, 0, now);
        } else {
            cfg_fallback(now);
        }
        break;
    case CFG_STEP_MONVER:
        if (s_pubx_used) {
            if (s_pubx_nmea_seen) {
                ESP_LOGW(TAG, "NMEA @115200 OK but UBX output unavailable; keep 115200 NMEA-only");
                s_cfg_baud_switched = false;
                cfg_fallback(now);
            } else {
                ESP_LOGW(TAG, "PUBX,41 path failed, revert 9600");
                uart_set_baudrate(UART_NUM_1, 9600);
                uart_flush_input(UART_NUM_1);
                s_data.baud = 9600;
                s_cfg_baud_switched = false;
                cfg_fallback(now);
            }
        } else {
            cfg_goto(CFG_STEP_RATE, 0, now);
        }
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
        /* BDS 在本颗片子（OTP: GPS GLO）不可用 -> 降级到实测唯一稳定组合 GPS+GLONASS，
         * 并锁存 s_bds_blocked + 同步 s_req_mode，避免后续重入配置时再次 NAK */
        if (s_effective_mode != GNSS_MODE_GPS_GLO && !s_gnss_fb_done) {
            s_gnss_fb_done = true;
            s_bds_blocked = true;
            s_effective_mode = GNSS_MODE_GPS_GLO;
            s_req_mode = GNSS_MODE_GPS_GLO;
            s_cfg_degraded = true;
            ESP_LOGW(TAG, "CFG-GNSS NAK -> fallback GPS+GLONASS（本模块 OTP 仅授权 GPS+GLO，已锁存 BDS 不可用）");
            cfg_goto(CFG_STEP_GNSS, 0, now);
        } else {
            s_cfg_degraded = true;
            ESP_LOGW(TAG, "CFG-GNSS NAK -> 保留模块当前星座配置");
            cfg_goto(CFG_STEP_NAV5, 0, now);
        }
        break;
    case CFG_STEP_NAV5:
        s_cfg_degraded = true;
        cfg_finish(now);
        break;
    case CFG_STEP_CLEAR:
        s_cfg_degraded = true;
        ESP_LOGW(TAG, "CFG-CFG clear failed, verify current baud before continuing");
        s_clear_try_115200 = false;
        uart_set_baudrate(UART_NUM_1, 9600);
        uart_flush_input(UART_NUM_1);
        s_data.baud = 9600;
        s_cfg_baud_switched = false;
        cfg_goto(CFG_STEP_CLEAR_VERIFY, 0, now);
        break;
    case CFG_STEP_CLEAR_VERIFY:
        if (!s_clear_try_115200) {
            s_clear_try_115200 = true;
            uart_set_baudrate(UART_NUM_1, 115200);
            uart_flush_input(UART_NUM_1);
            s_data.baud = 115200;
            ESP_LOGW(TAG, "CFG-CFG verify @9600 failed, retry @115200");
            cfg_goto(CFG_STEP_CLEAR_VERIFY, 0, now);
        } else {
            ESP_LOGE(TAG, "CFG-CFG verify failed @9600 and @115200");
            cfg_fallback(now);
        }
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

    if (s_cfg_step == CFG_STEP_PUBX) {
        if (now >= s_cfg_next_ms) {
            /* PUBX,41 已发出：尝试 115200；若模块未接受，MON-VER 超时后回退 9600 */
            uart_set_baudrate(UART_NUM_1, 115200);
            uart_flush_input(UART_NUM_1);
            s_data.baud = 115200;
            s_cfg_baud_switched = true;
            s_pubx_used = true;
            ESP_LOGI(TAG, "PUBX,41 re-send @115200 to ensure in/out UBX");
            nmea_send_pubx41();
            cfg_goto(CFG_STEP_PRT, 200, now);
        }
        return;
    }
    if (s_cfg_step == CFG_STEP_GNSSPOLL) {
        /* 回读不是 ACK 语义：等到模板到达，或超时后直接跳过 CFG-GNSS 下发 */
        if (s_gnss_tpl_valid) {
            ESP_LOGI(TAG, "CFG-GNSS template cached (%u blocks)", (unsigned)s_gnss_tpl_n);
            cfg_goto(CFG_STEP_RATE, 0, now);
        } else if (now - s_cfg_sent_ms > GNSS_CFG_ACK_TIMEOUT_MS) {
            ESP_LOGW(TAG, "CFG-GNSS readback timeout, 跳过 CFG-GNSS（保留模块默认星座）");
            s_cfg_degraded = true;
            cfg_goto(CFG_STEP_RATE, 0, now);
        }
        return;
    }
    if (s_cfg_step == CFG_STEP_MONVER || s_cfg_step == CFG_STEP_UBX_PROBE ||
        s_cfg_step == CFG_STEP_CLEAR_VERIFY) {
        if (s_monver_ms >= s_cfg_sent_ms && s_monver_ms != 0) {
            cfg_on_success(now);
        } else if (now - s_cfg_sent_ms > GNSS_CFG_ACK_TIMEOUT_MS) {
            if (s_cfg_retry < GNSS_CFG_RETRY) {
                s_cfg_retry++;
                s_cfg_need_send = true;
                s_cfg_next_ms = now;
            } else {
                cfg_on_failure(now, true);
            }
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
    s_clear_try_115200 = false;
    s_pubx_tried = false;
    s_pubx_used = false;
    s_pubx_nmea_seen = false;
    s_effective_mode = s_req_mode;
    s_effective_rate = gnss_rate_clamp(s_req_rate_hz, s_effective_mode);
    if (s_effective_rate != s_req_rate_hz) {
        ESP_LOGW(TAG, "rate %uHz 超出 %s 上限，实际按 %uHz 配置",
                 (unsigned)s_req_rate_hz, gnss_mode_name(s_effective_mode),
                 (unsigned)s_effective_rate);
    }
    s_data.cfg_state = GNSS_CFG_CONFIGURING;

    if (s_selftest_done) {
        bool ubx9600_ok = s_st_ubx_9600 || s_st_pubx_9600 || s_st_force_9600;
        bool ubx115200_ok = s_st_ubx_115200 || s_st_pubx_115200;
        if (!ubx9600_ok && !ubx115200_ok) {
            cfg_fallback(now);
        } else if (s_data.baud == 115200) {
            cfg_goto(CFG_STEP_MONVER, 0, now);
        } else {
            cfg_goto(CFG_STEP_PRT, 0, now);
        }
        return;
    }

    if (s_data.baud == 115200) {
        /* 模块开机就在 115200 -> 说明 BBR/Flash 里还存着旧固件保存的配置。
         * 先清一次，保证下次上电回到 9600 默认；本次继续在 115200 上配置。 */
        ESP_LOGW(TAG, "boot @115200: legacy saved config detected, clearing once (no save afterwards)");
        cfg_goto(CFG_STEP_CLEAR, 0, now);
    } else {
        cfg_goto(CFG_STEP_UBX_PROBE, 0, now);
    }
}

/* ==================== 一次性 GNSS 自检 ==================== */
static const char *yes_no(bool v)
{
    return v ? "YES" : "NO";
}

/* 在指定波特率下发送 UBX-MON-VER poll，阻塞等待响应（仅启动自检使用） */
/* 诊断开关：逐字节慢发（字节间 2ms），用于排除“线路容性/串联电阻导致边沿变形”
 * 造成的帧丢弃 —— 快发失败但慢发成功，基本就是电气问题 */
static bool s_probe_slow = false;

static bool ubx_probe_at(uint32_t baud, uint32_t timeout_ms)
{
    uart_set_baudrate(UART_NUM_1, baud);
    uart_flush_input(UART_NUM_1);
    vTaskDelay(pdMS_TO_TICKS(30));
    s_data.baud = baud;
    s_line_len = 0;
    s_in_ubx = false;
    s_monver_ms = 0;
    uint16_t sync0 = s_cnt_ubx_sync;
    uint16_t nmea0 = s_cnt_gga + s_cnt_rmc;
    if (s_probe_slow) {
        uint8_t f[8];
        size_t fn = ubx_build(0x0A, 0x04, NULL, 0, f, sizeof(f));
        for (size_t i = 0; i < fn; i++) {
            uart_write_bytes(UART_NUM_1, &f[i], 1);
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    } else {
        ubx_send(0x0A, 0x04, NULL, 0);
    }

    uint64_t start = now_ms();
    uint8_t buf[256];
    uint8_t first[16];
    int first_n = 0;
    uint32_t rx_total = 0;
    while (now_ms() - start < timeout_ms) {
        int len = uart_read_bytes(UART_NUM_1, buf, sizeof(buf), pdMS_TO_TICKS(50));
        for (int i = 0; i < len; i++) {
            if (first_n < (int)sizeof(first)) {
                first[first_n++] = buf[i];
            }
            feed_byte(buf[i]);
        }
        rx_total += (uint32_t)len;
        if (s_monver_ms != 0) {
            ESP_LOGI(TAG, "probe @%lu: MON-VER 应答到达（rx=%lu B, sync+%u）",
                     (unsigned long)baud, (unsigned long)rx_total,
                     (unsigned)(s_cnt_ubx_sync - sync0));
            return true;
        }
    }

    /* 失败：把本窗口收到的原始字节吐出来，用于区分
     *   · 收到 NMEA 文本无 UBX  -> 模块听了但没回（或 UBX 输入被禁）
     *   · 全是乱码              -> 本端波特率与模块不匹配
     *   · 一字节都没有          -> 模块根本没在发（断电/复位）     */
    char hex[64] = { 0 };
    int off = 0;
    for (int i = 0; i < first_n && off < (int)sizeof(hex) - 4; i++) {
        off += snprintf(hex + off, sizeof(hex) - (size_t)off, "%02X ", first[i]);
        if (i == 7 && first_n > 8) {
            off += snprintf(hex + off, sizeof(hex) - (size_t)off, "... ");
            break;
        }
    }
    ESP_LOGW(TAG, "probe @%lu fail: rx=%lu B, sync+%u, nmea+%u, first: %s",
             (unsigned long)baud, (unsigned long)rx_total,
             (unsigned)(s_cnt_ubx_sync - sync0),
             (unsigned)(s_cnt_gga + s_cnt_rmc - nmea0), hex);
    return false;
}

/* 自检 0：TX 引脚电平回读——不用仪表就能发现“TX 被拉低/短路/被其他主机强驱”
 *
 * ESP32 的 pad 回读走输入通道：若外部强拉或对地短路，回读值与驱动值不一致。
 * 需先让 UART 放弃该引脚，测完再交回。
 */
static void tx_pad_selftest(void)
{
    int hi = -1, lo = -1, pu = -1;
    gpio_reset_pin(PIN_GNSS_TX);      /* 先释放可能被保留的引脚，避免 gpio: conflict 警告 */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_GNSS_TX),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&io) == ESP_OK) {
        gpio_set_level(PIN_GNSS_TX, 1);
        vTaskDelay(pdMS_TO_TICKS(2));
        hi = gpio_get_level(PIN_GNSS_TX);
        gpio_set_level(PIN_GNSS_TX, 0);
        vTaskDelay(pdMS_TO_TICKS(2));
        lo = gpio_get_level(PIN_GNSS_TX);

        /* 浮空 + 上拉：被外部强下拉或短路到地会读到 0 */
        gpio_config_t in = {
            .pin_bit_mask = (1ULL << PIN_GNSS_TX),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        if (gpio_config(&in) == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(2));
            pu = gpio_get_level(PIN_GNSS_TX);
        }
    }

    /* 关键：必须 gpio_reset_pin 才能把引脚交回 UART 复用，
     * 否则 uart_set_pin() 会报 “GPIO 17 is not usable” 并失败 -> TX 彻底断开 */
    gpio_reset_pin(PIN_GNSS_TX);
    ESP_LOGW(TAG, "SELFTEST TX pad(GPIO%d): drive1->%d drive0->%d pullup->%d  %s",
             PIN_GNSS_TX, hi, lo, pu,
             (hi == 1 && lo == 0 && pu == 1) ? "pad 正常" : "异常: 被拉低/短路/被强驱");
}

/* 读空 UART 并逐字节喂给解析器（自检阻塞等待期间必须持续排水，
 * 否则 s_cnt_gga 等计数器不会增长，测试会得出错误结论） */
static void rx_drain(uint32_t ms)
{
    uint8_t buf[128];
    uint64_t end = now_ms() + ms;
    while (now_ms() < end) {
        int len = uart_read_bytes(UART_NUM_1, buf, sizeof(buf), pdMS_TO_TICKS(20));
        for (int i = 0; i < len; i++) {
            feed_byte(buf[i]);
        }
    }
}

/* 自检 1：UART1 内部环回——验证驱动/发送 FIFO 通路（不经过物理引脚） */
static void uart_loopback_selftest(void)
{
    uint8_t tx[8];
    size_t n = ubx_build(0x0A, 0x04, NULL, 0, tx, sizeof(tx));
    if (n == 0) {
        return;
    }
    rx_drain(100);
    uart_set_loop_back(UART_NUM_1, true);
    uart_flush_input(UART_NUM_1);
    uart_write_bytes(UART_NUM_1, tx, n);
    uint8_t rx[32] = { 0 };
    int got = uart_read_bytes(UART_NUM_1, rx, sizeof(rx), pdMS_TO_TICKS(200));
    uart_set_loop_back(UART_NUM_1, false);
    uart_flush_input(UART_NUM_1);

    bool echo = false;
    for (int i = 0; i + (int)n <= got; i++) {
        if (memcmp(&rx[i], tx, n) == 0) {
            echo = true;
            break;
        }
    }
    char hex[48] = { 0 };
    int off = 0;
    for (int i = 0; i < got && i < 8; i++) {
        off += snprintf(hex + off, sizeof(hex) - (size_t)off, "%02X ", rx[i]);
    }
    ESP_LOGI(TAG, "SELFTEST UART loopback=%s (tx %u B, rx %d B, rx: %s) [S3 环回会混入外部 RX，仅供参考]",
             yes_no(echo), (unsigned)n, got, hex);
}

/* 自检 2：NMEA 发送效果——判定“字节是否真的到达模块 RX”
 *
 * 手法：用 $PUBX,41 请求模块把 UART1 切到 115200。若字节到达，模块会真的换速率，
 * 此时仍工作在 9600 的本端就收不到可解析的 NMEA，GGA 计数停止增长。
 * 若固件 TX 断线/被其他主机抢线，模块收不到，GGA 依旧 1 Hz 继续增长。
 * 本测试不依赖 UBX，也不依赖 ACK。
 */
static bool nmea_tx_effect_selftest(void)
{
    /* 三段测量：9600 下发切换命令后，看 NMEA 是否从 9600 消失、又在 115200 出现、
     * 最后发回 9600 命令后回到 9600。TX 断线时模块不会动，三段都会在 9600 上正常。 */
    rx_drain(300);
    uint16_t a0 = s_cnt_gga;
    nmea_send_pubx41_cfg(115200, 0x0007, 0x0003);
    rx_drain(2000);
    uint16_t n9600 = (uint16_t)(s_cnt_gga - a0);

    uart_set_baudrate(UART_NUM_1, 115200);
    uart_flush_input(UART_NUM_1);
    s_data.baud = 115200;
    s_line_len = 0;
    s_in_ubx = false;
    uint16_t b0 = s_cnt_gga;
    rx_drain(2000);
    uint16_t n115200 = (uint16_t)(s_cnt_gga - b0);

    /* 回 9600：本端已在 115200，先发 PUBX 让模块也回 9600，再把本端切回 */
    nmea_send_pubx41_cfg(9600, 0x0007, 0x0003);
    rx_drain(500);
    uart_set_baudrate(UART_NUM_1, 9600);
    uart_flush_input(UART_NUM_1);
    s_data.baud = 9600;
    s_line_len = 0;
    s_in_ubx = false;
    uint16_t c0 = s_cnt_gga;
    rx_drain(2000);
    uint16_t nback = (uint16_t)(s_cnt_gga - c0);

    bool reached = (n9600 == 0) && (n115200 > 0);
    ESP_LOGW(TAG, "SELFTEST NMEA TX: GGA@9600=%u @115200=%u back@9600=%u -> 字节到达模块=%s",
             (unsigned)n9600, (unsigned)n115200, (unsigned)nback, yes_no(reached));
    return reached;
}

/* 一个版本内验证：
 *   UART 环回 → NMEA 发送效果 → UBX@9600 → PUBX,41(0003)@9600 → PUBX,41(0007)@9600
 *   → PUBX,41(0003)@115200 → PUBX,41(0007)@115200
 * 并输出一行 SELFTEST SUMMARY。
 */
static void gnss_selftest(void)
{
    ESP_LOGI(TAG, "SELFTEST start (NEO-M8N-0-01 / PROTVER 15.00)");

    /* 先区分“固件发不出去”与“模块收不到”：两级自检
     * （TX pad 回读已在 gnss_init() 中、UART 接管引脚之前完成）*/
    uart_loopback_selftest();
    nmea_tx_effect_selftest();

    s_st_ubx_9600 = ubx_probe_at(9600, 700);
    ESP_LOGI(TAG, "SELFTEST UBX@9600=%s (RX 侧累计 UBX sync=%u)",
             yes_no(s_st_ubx_9600), (unsigned)s_cnt_ubx_sync);

    /* 如果快发没应答，再试一次逐字节慢发：快失败/慢成功 => 电气（边沿/容性/串联电阻）问题 */
    if (!s_st_ubx_9600) {
        s_probe_slow = true;
        bool slow_ok = ubx_probe_at(9600, 800);
        s_probe_slow = false;
        ESP_LOGW(TAG, "SELFTEST UBX@9600 slow-byte=%s", yes_no(slow_ok));
        s_st_ubx_9600 = slow_ok;
    }

    if (!s_st_ubx_9600) {
        nmea_send_pubx41_cfg(9600, 0x0003, 0x0003);
        vTaskDelay(pdMS_TO_TICKS(500));
        s_st_pubx_9600 = ubx_probe_at(9600, 700);
        ESP_LOGI(TAG, "SELFTEST PUBX@9600(0003)->UBX=%s", yes_no(s_st_pubx_9600));
        if (!s_st_pubx_9600) {
            nmea_send_pubx41_cfg(9600, 0x0007, 0x0007);
            vTaskDelay(pdMS_TO_TICKS(500));
            s_st_pubx_9600 = ubx_probe_at(9600, 700);
            ESP_LOGI(TAG, "SELFTEST PUBX@9600(0007)->UBX=%s", yes_no(s_st_pubx_9600));
        }
    }

    /* 115200 验证：先让模块切波特率，再发 PUBX,41 确保协议掩码 */
    s_pubx_nmea_seen = false;
    nmea_send_pubx41_cfg(115200, 0x0003, 0x0003);
    vTaskDelay(pdMS_TO_TICKS(300));
    s_st_ubx_115200 = ubx_probe_at(115200, 900);
    ESP_LOGI(TAG, "SELFTEST PUBX@115200(0003)->UBX=%s", yes_no(s_st_ubx_115200));
    if (!s_st_ubx_115200) {
        nmea_send_pubx41_cfg(115200, 0x0007, 0x0007);
        vTaskDelay(pdMS_TO_TICKS(500));
        s_st_ubx_115200 = ubx_probe_at(115200, 900);
        ESP_LOGI(TAG, "SELFTEST PUBX@115200(0007)->UBX=%s", yes_no(s_st_ubx_115200));
    }
    s_st_pubx_115200 = s_st_ubx_115200;
    s_st_nmea_115200 = s_pubx_nmea_seen;

    /* 最后再试一次“强制 UBX 输入”：输入仅 UBX，输出保留 NMEA+UBX。
     * 若成功，说明模块需要 inProto=0001 才接受 UBX；若失败，NMEA 输出仍在，可安全回退。 */
    if (!s_st_ubx_9600 && !s_st_pubx_9600 && !s_st_ubx_115200 && !s_st_pubx_115200) {
        nmea_send_pubx41_cfg(9600, 0x0003, 0x0003);   /* 先恢复 9600 + NMEA 输入 */
        vTaskDelay(pdMS_TO_TICKS(300));
        uart_set_baudrate(UART_NUM_1, 9600);
        uart_flush_input(UART_NUM_1);
        s_data.baud = 9600;
        nmea_send_pubx41_cfg(9600, 0x0001, 0x0003);   /* 输入仅 UBX，输出 NMEA+UBX */
        vTaskDelay(pdMS_TO_TICKS(500));
        s_st_force_9600 = ubx_probe_at(9600, 700);
        ESP_LOGI(TAG, "SELFTEST PUBX@9600(0001/0003)->UBX=%s", yes_no(s_st_force_9600));
        s_st_nmea_115200 = false;   /* 已切回 9600；失败时 NMEA 仍在 9600 */
    }

    bool ubx9600_ok = s_st_ubx_9600 || s_st_pubx_9600 || s_st_force_9600;
    bool ubx115200_ok = s_st_ubx_115200 || s_st_pubx_115200;
    uint32_t use_baud;
    if (ubx115200_ok) {
        use_baud = 115200;
    } else if (ubx9600_ok) {
        use_baud = 9600;
    } else if (s_st_nmea_115200) {
        use_baud = 115200;
    } else {
        use_baud = 9600;
    }
    uart_set_baudrate(UART_NUM_1, use_baud);
    uart_flush_input(UART_NUM_1);
    s_data.baud = use_baud;
    s_selftest_done = true;

    ESP_LOGI(TAG, "SELFTEST SUMMARY: UBX9600=%s PUBX9600=%s FORCE9600=%s UBX115200=%s PUBX115200=%s NMEA115200=%s -> use %lu (RX UBX sync=%u)",
             yes_no(s_st_ubx_9600), yes_no(s_st_pubx_9600), yes_no(s_st_force_9600),
             yes_no(s_st_ubx_115200), yes_no(s_st_pubx_115200),
             yes_no(s_st_nmea_115200), (unsigned long)use_baud, (unsigned)s_cnt_ubx_sync);
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
        ESP_LOGI(TAG, "SAT: used=%u pvt=%u tracked=%u view=%u GPS=%u GLO=%u BDS=%u GAL=%u QZSS=%u maxCN0=%u GSV=%u NAV-SAT=%u",
                 (unsigned)s_sat.used,
                 (unsigned)s_data.sats,
                 (unsigned)s_data.sats_tracked,
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
                if (!s_selftest_done && GNSS_SELFTEST_FULL) {
                    gnss_selftest();
                }
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
                s_selftest_done = false;
                s_last_navsat_ms = 0;
                s_last_navpvt_ms = 0;
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
                        /* 只有“请求值真变了”才重跑配置，避免开机时 NVS 回填
                         * 触发多轮重复配置（重复写 Flash、cfg_state 在 OK/DEGRADED 间跳） */
                        if (s_req_rate_dirty) {
                            s_req_rate_dirty = false;
                            if (s_req_rate_hz != s_effective_rate) {
                                s_cfg_degraded = false;
                                s_rate_fb_done = false;
                                s_effective_rate = gnss_rate_clamp(s_req_rate_hz, s_effective_mode);
                                s_msg_idx = 0;
                                cfg_goto(CFG_STEP_RATE, 0, now);
                            }
                        } else if (s_req_mode_dirty) {
                            s_req_mode_dirty = false;
                            if (s_req_mode != s_effective_mode) {
                                s_cfg_degraded = false;
                                s_gnss_fb_done = false;
                                s_effective_mode = s_req_mode;
                                /* 模式切换可能改变速率上限：GPS_ONLY 允许 10Hz，
                                 * GPS+GLO 只允许 5Hz。按请求值重新 clamp，
                                 * 既降频也升频。 */
                                uint8_t new_rate = gnss_rate_clamp(s_req_rate_hz, s_effective_mode);
                                if (new_rate != s_effective_rate) {
                                    s_effective_rate = new_rate;
                                    s_msg_idx = 0;
                                    cfg_goto(CFG_STEP_RATE, 0, now);
                                } else {
                                    cfg_goto(CFG_STEP_GNSS, 0, now);
                                }
                            }
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
            s_data.log_writes = s_log_writes;
            s_data.log_drops = s_log_drops;
            s_stat_ms = now;
        }

        /* 周期打印卫星/串口统计，便于无屏幕时定位“为什么卫星页没数据” */
        if (locked && now - s_last_sat_log_ms >= 5000) {
            s_last_sat_log_ms = now;
            log_sat_summary();
        }
        if (now - s_last_nmea_log_ms >= 10000) {
            s_last_nmea_log_ms = now;
            ESP_LOGI(TAG, "COUNTERS: GGA=%u RMC=%u GSA=%u GSV=%u | NAV-PVT=%u NAV-SAT=%u | UBX frame=%u sync=%u | crc=%u | cfg_step=%s",
                     (unsigned)s_cnt_gga, (unsigned)s_cnt_rmc, (unsigned)s_cnt_gsa, (unsigned)s_cnt_gsv,
                     (unsigned)s_cnt_nav_pvt, (unsigned)s_cnt_nav_sat,
                     (unsigned)s_cnt_ubx_frames, (unsigned)s_cnt_ubx_sync,
                     (unsigned)s_data.crc_err, cfg_step_name(s_cfg_step));
        }
    }
}

/* ==================== 接口 ==================== */
esp_err_t gnss_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    s_log_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL || s_log_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_boot_ms = now_ms();
    s_stat_ms = s_boot_ms;
    s_data.rate_hz = 10;

    /* 先验证 UBX 帧构造（CK_A/CK_B），历史上这里出错会让全部 UBX 配置静默失效 */
    ubx_ck_selfcheck();
    ubx_parse_selfcheck();

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

    /* TX pad 回读必须在 UART 接管引脚之前做（先 gpio_config 再用 uart_set_pin 会报
     * “GPIO 17 is not usable” 并导致 TX 彻底断开）；测完 gpio_reset_pin 交回复用 */
    tx_pad_selftest();

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
    ESP_LOGI(TAG, "GNSS init ok: UART1 TX=%d RX=%d, LDO EN=%d, baud 9600/115200, NMEA+UBX",
             PIN_GNSS_TX, PIN_GNSS_RX, PIN_GNSS_LDO_EN);
    return ESP_OK;
}

/* 仅供电模式：给模块上电，但完全不碰 UART1（不装驱动、不发字节、不建任务），
 * 并把 TX/RX 引脚释放成高阻输入，典型用途是外挂 u-center 独占总线排障。 */
esp_err_t gnss_power_only_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    s_log_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL || s_log_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* EN：LDO 使能（高电平）——模块保持上电，仍按自身配置自主输出 NMEA */
    gpio_config_t en = {
        .pin_bit_mask = (1ULL << PIN_GNSS_LDO_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&en), TAG, "gpio_config(EN) failed");
    gpio_set_level(PIN_GNSS_LDO_EN, 1);

    /* 释放 UART 引脚：高阻输入，既不驱动模块 RX，也不与 u-center 抢总线。
     * 注意不能用 UART_PIN_NO_CHANGE/uarts：必须先不安装驱动，再改引脚方向。 */
    gpio_config_t rel = {
        .pin_bit_mask = (1ULL << PIN_GNSS_TX) | (1ULL << PIN_GNSS_RX),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&rel), TAG, "gpio_config(UART release) failed");

    s_boot_ms = now_ms();
    s_stat_ms = s_boot_ms;
    ESP_LOGW(TAG, "GNSS UART DISABLED (GNSS_DISABLE_FOR_UCENTER=1): LDO EN=%d 已上电，"
                  "UART1 未安装，TX%d/RX%d 已释放为高阻。请用 u-center 直接连接模块串口。",
             PIN_GNSS_LDO_EN, PIN_GNSS_TX, PIN_GNSS_RX);
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
        /* NAV-SAT 新鲜时以逐星 svUsed 统计为准，保证 health/状态栏与 SAT INFO 同源；
         * 否则保留 GGA/NAV-PVT 的 numSV（NAV-SAT 不可用时的回退）。 */
        if (s_last_navsat_ms != 0 && (now_ms() - s_last_navsat_ms) <= 2000) {
            out->sats = s_sat.used;
        }
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
    if (s_log_mutex && xSemaphoreTake(s_log_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
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
        xSemaphoreGive(s_log_mutex);
    }
}

void gnss_set_update_rate(uint8_t hz)
{
    if (hz == 18) {
        hz = GNSS_RATE_MAX_SINGLE_HZ;   /* 本模块最高 10 Hz */
    }
    if (hz != 1 && hz != 2 && hz != 5 && hz != 10) {
        hz = 10;
    }
    if (hz == s_req_rate_hz) {
        return;                     /* 值没变，不必重跑配置 */
    }
    s_req_rate_hz = hz;
    s_req_rate_dirty = true;
    if (s_data.cfg_state == GNSS_CFG_FALLBACK) {
        ESP_LOGW(TAG, "setting queued: rate=%uHz, but UBX unavailable (FALLBACK)", (unsigned)hz);
    }
}

void gnss_set_gnss_mode(uint8_t mode)
{
    if (mode > GNSS_MODE_GPS_ONLY) {
        mode = GNSS_MODE_GPS_GLO_BDS;
    }
    if (mode == s_req_mode) {
        return;                     /* 值没变，不必重跑配置 */
    }
    s_req_mode = mode;
    s_req_mode_dirty = true;
    if (s_data.cfg_state == GNSS_CFG_FALLBACK) {
        ESP_LOGW(TAG, "setting queued: mode=%s, but UBX unavailable (FALLBACK)", gnss_mode_name(mode));
    }
}

uint8_t gnss_get_update_rate(void)
{
    return s_effective_rate;
}

uint8_t gnss_get_gnss_mode(void)
{
    return s_effective_mode;
}
