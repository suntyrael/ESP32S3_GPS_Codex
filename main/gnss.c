#include "gnss.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_bit_defs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "config.h"
#include "settings_store.h"

#define NMEA_MAX_LINE 128

static const char *TAG = "gnss";
static uart_port_t s_uart = CONFIG_GNSS_UART_PORT;
static char s_line_buf[NMEA_MAX_LINE];
static size_t s_line_len;
static uint16_t s_sat_in_use[GNSS_MAX_SATELLITES];
static size_t s_sat_in_use_count;
static SemaphoreHandle_t s_uart_lock;
static StaticSemaphore_t s_uart_lock_buffer;

static time_t tm_to_epoch(const struct tm *tm) {
    if (!tm) {
        return 0;
    }
    int year = tm->tm_year + 1900;
    int month = tm->tm_mon + 1;
    int day = tm->tm_mday;
    if (month <= 2) {
        year -= 1;
        month += 12;
    }
    long days = (long)(365 * year) + year / 4 - year / 100 + year / 400 + (153 * (month - 3) + 2) / 5 + day - 719469;
    return (time_t)days * 86400 + tm->tm_hour * 3600 + tm->tm_min * 60 + tm->tm_sec;
}

static inline uint8_t checksum_nmea(const char *payload) {
    uint8_t sum = 0;
    while (*payload && *payload != '*') {
        sum ^= (uint8_t)(*payload++);
    }
    return sum;
}

static gnss_constellation_t constellation_from_talker(const char *talker) {
    if (!talker) {
        return GNSS_CONSTELLATION_UNKNOWN;
    }
    if (strncmp(talker, "GP", 2) == 0) {
        return GNSS_CONSTELLATION_GPS;
    }
    if (strncmp(talker, "GL", 2) == 0) {
        return GNSS_CONSTELLATION_GLONASS;
    }
    if (strncmp(talker, "GA", 2) == 0) {
        return GNSS_CONSTELLATION_GALILEO;
    }
    if (strncmp(talker, "BD", 2) == 0 || strncmp(talker, "GB", 2) == 0) {
        return GNSS_CONSTELLATION_BEIDOU;
    }
    return GNSS_CONSTELLATION_UNKNOWN;
}

static bool parse_rmc_datetime(const char *time_field, const char *date_field, time_t *timestamp) {
    if (!time_field || !date_field || strlen(time_field) < 6 || strlen(date_field) < 6) {
        return false;
    }
    struct tm tm = {0};
    tm.tm_hour = (time_field[0] - '0') * 10 + (time_field[1] - '0');
    tm.tm_min = (time_field[2] - '0') * 10 + (time_field[3] - '0');
    tm.tm_sec = (time_field[4] - '0') * 10 + (time_field[5] - '0');
    tm.tm_mday = (date_field[0] - '0') * 10 + (date_field[1] - '0');
    tm.tm_mon = ((date_field[2] - '0') * 10 + (date_field[3] - '0')) - 1;
    tm.tm_year = ((date_field[4] - '0') * 10 + (date_field[5] - '0')) + 100; // 2000+
    tm.tm_isdst = 0;
    if (tm.tm_mon < 0 || tm.tm_mon > 11) {
        return false;
    }
    time_t epoch = tm_to_epoch(&tm);
    if (epoch <= 0) {
        return false;
    }
    if (timestamp) {
        *timestamp = epoch;
    }
    return true;
}

static double parse_coord(const char *value, const char hemi) {
    if (!value || !value[0]) {
        return 0.0;
    }
    double raw = atof(value);
    int degrees = (int)(raw / 100.0);
    double minutes = raw - (degrees * 100.0);
    double decimal = degrees + minutes / 60.0;
    if (hemi == 'S' || hemi == 'W') {
        decimal = -decimal;
    }
    return decimal;
}

static gnss_satellite_t *find_or_alloc_sat(gnss_state_t *state, uint8_t id, gnss_constellation_t c) {
    for (int i = 0; i < GNSS_MAX_SATELLITES; ++i) {
        if (state->satellites[i].id == id || state->satellites[i].id == 0) {
            state->satellites[i].id = id;
            state->satellites[i].constellation = c;
            return &state->satellites[i];
        }
    }
    return &state->satellites[GNSS_MAX_SATELLITES - 1];
}

static bool sat_in_use(uint8_t id) {
    for (size_t i = 0; i < s_sat_in_use_count; ++i) {
        if (s_sat_in_use[i] == id) {
            return true;
        }
    }
    return false;
}

static void update_sat_statuses(gnss_state_t *state) {
    for (int i = 0; i < GNSS_MAX_SATELLITES; ++i) {
        gnss_satellite_t *sat = &state->satellites[i];
        if (sat->id == 0) {
            continue;
        }
        if (sat_in_use(sat->id)) {
            sat->status = GNSS_SAT_STATUS_USED;
        } else if (sat->cn0_dbhz > 0.0f) {
            sat->status = GNSS_SAT_STATUS_TRACKING;
        } else {
            sat->status = GNSS_SAT_STATUS_SEARCHING;
        }
    }
}

static void handle_gga(gnss_state_t *state, char *payload) {
    char *fields[15] = {0};
    int count = 0;
    char *token = payload;
    while (token && count < 15) {
        fields[count++] = token;
        token = strchr(token, ',');
        if (token) {
            *token++ = '\0';
        }
    }
    if (count < 10) {
        return;
    }
    state->latitude_deg = parse_coord(fields[1], fields[2] ? fields[2][0] : 'N');
    state->longitude_deg = parse_coord(fields[3], fields[4] ? fields[4][0] : 'E');
    state->fix_valid = fields[5] && fields[5][0] > '0';
    state->sats_in_use = fields[6] ? (uint8_t)atoi(fields[6]) : 0;
    state->hdop = fields[7] ? atof(fields[7]) : 99.9f;
    state->altitude_m = fields[8] ? atof(fields[8]) : 0.0f;
}

static void handle_rmc(gnss_state_t *state, char *payload) {
    char *fields[15] = {0};
    int count = 0;
    char *token = payload;
    while (token && count < 15) {
        fields[count++] = token;
        token = strchr(token, ',');
        if (token) {
            *token++ = '\0';
        }
    }
    if (count < 10) {
        return;
    }
    state->fix_valid = fields[1] && fields[1][0] == 'A';
    state->latitude_deg = parse_coord(fields[2], fields[3] ? fields[3][0] : 'N');
    state->longitude_deg = parse_coord(fields[4], fields[5] ? fields[5][0] : 'E');
    double speed_knots = fields[6] ? atof(fields[6]) : 0.0;
    state->speed_kmh = (float)(speed_knots * 1.852);
    if (fields[0] && fields[8]) {
        time_t timestamp;
        if (parse_rmc_datetime(fields[0], fields[8], &timestamp)) {
            state->timestamp_utc = timestamp;
        }
    }
}

static void handle_gsa(gnss_state_t *state, char *payload) {
    char *fields[18] = {0};
    int count = 0;
    char *token = payload;
    while (token && count < 18) {
        fields[count++] = token;
        token = strchr(token, ',');
        if (token) {
            *token++ = '\0';
        }
    }
    if (count < 17) {
        return;
    }
    s_sat_in_use_count = 0;
    for (int i = 2; i < 14 && s_sat_in_use_count < GNSS_MAX_SATELLITES; ++i) {
        if (fields[i] && fields[i][0]) {
            s_sat_in_use[s_sat_in_use_count++] = (uint16_t)atoi(fields[i]);
        }
    }
    state->pdop = fields[14] ? atof(fields[14]) : 99.9f;
    state->hdop = fields[15] ? atof(fields[15]) : state->hdop;
    state->vdop = fields[16] ? atof(fields[16]) : 99.9f;
    update_sat_statuses(state);
}

static void handle_gsv(gnss_state_t *state, const char *talker, char *payload) {
    char *fields[20] = {0};
    int count = 0;
    char *token = payload;
    while (token && count < 20) {
        fields[count++] = token;
        token = strchr(token, ',');
        if (token) {
            *token++ = '\0';
        }
    }
    if (count < 4) {
        return;
    }
    state->sats_in_view = fields[3] ? (uint8_t)atoi(fields[3]) : state->sats_in_view;
    gnss_constellation_t constellation = constellation_from_talker(talker);
    for (int i = 4; i + 3 < count; i += 4) {
        uint8_t id = fields[i] ? (uint8_t)atoi(fields[i]) : 0;
        if (!id) {
            continue;
        }
        gnss_satellite_t *sat = find_or_alloc_sat(state, id, constellation);
        sat->elevation_deg = fields[i + 1] ? atof(fields[i + 1]) : 0.0f;
        sat->azimuth_deg = fields[i + 2] ? atof(fields[i + 2]) : 0.0f;
        sat->cn0_dbhz = fields[i + 3] ? atof(fields[i + 3]) : 0.0f;
        sat->status = sat_in_use(id) ? GNSS_SAT_STATUS_USED : GNSS_SAT_STATUS_TRACKING;
    }
}

static void parse_sentence(gnss_state_t *state, char *line) {
    if (line[0] != '$') {
        return;
    }
    char *checksum = strchr(line, '*');
    if (checksum) {
        *checksum = '\0';
    }
    char *payload = line + 1;
    char *comma = strchr(payload, ',');
    if (!comma) {
        return;
    }
    *comma = '\0';
    const char *talker = payload;
    const char *sentence = comma + 1;
    if (strncmp(talker + 2, "GGA", 3) == 0) {
        handle_gga(state, (char *)sentence);
    } else if (strncmp(talker + 2, "RMC", 3) == 0) {
        handle_rmc(state, (char *)sentence);
    } else if (strncmp(talker + 2, "GSA", 3) == 0) {
        handle_gsa(state, (char *)sentence);
    } else if (strncmp(talker + 2, "GSV", 3) == 0) {
        handle_gsv(state, talker, (char *)sentence);
    }
}

static void consume_byte(gnss_state_t *state, uint8_t byte) {
    if (byte == '\r') {
        return;
    }
    if (byte == '\n') {
        if (s_line_len > 0) {
            s_line_buf[s_line_len] = '\0';
            parse_sentence(state, s_line_buf);
            s_line_len = 0;
        }
        return;
    }
    if (s_line_len < sizeof(s_line_buf) - 1) {
        s_line_buf[s_line_len++] = (char)byte;
    } else {
        s_line_len = 0;
    }
}

static bool read_line_blocking(char *out, size_t max_len, TickType_t timeout) {
    TickType_t start = xTaskGetTickCount();
    size_t len = 0;
    while ((xTaskGetTickCount() - start) < timeout) {
        uint8_t byte;
        int read = uart_read_bytes(s_uart, &byte, 1, pdMS_TO_TICKS(20));
        if (read > 0) {
            if (byte == '\n') {
                if (len < max_len) {
                    out[len] = '\0';
                    return true;
                }
                return false;
            }
            if (byte != '\r' && len + 1 < max_len) {
                out[len++] = (char)byte;
            }
        }
    }
    return false;
}

static bool wait_ack_locked(const char *keyword, TickType_t timeout) {
    char line[NMEA_MAX_LINE];
    TickType_t start = xTaskGetTickCount();
    while (1) {
        TickType_t now = xTaskGetTickCount();
        if ((now - start) >= timeout) {
            break;
        }
        TickType_t remaining = timeout - (now - start);
        if (!read_line_blocking(line, sizeof(line), remaining)) {
            break;
        }
        if (strstr(line, keyword)) {
            ESP_LOGI(TAG, "GNSS ACK %s", keyword);
            return true;
        }
    }
    ESP_LOGW(TAG, "GNSS ACK timeout: %s", keyword);
    return false;
}

static void send_command_locked(const char *payload) {
    char sentence[64];
    uint8_t checksum = checksum_nmea(payload);
    snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n", payload, checksum);
    uart_write_bytes(s_uart, sentence, strlen(sentence));
}

static bool send_command_with_ack(const char *payload, const char *ack, TickType_t timeout) {
    if (!s_uart_lock) {
        return false;
    }
    if (xSemaphoreTake(s_uart_lock, pdMS_TO_TICKS(200)) != pdTRUE) {
        return false;
    }
    send_command_locked(payload);
    bool ok = true;
    if (ack) {
        ok = wait_ack_locked(ack, timeout);
    }
    xSemaphoreGive(s_uart_lock);
    return ok;
}

static void send_ubx_locked(uint8_t cls, uint8_t id, const uint8_t *payload, size_t len) {
    uint8_t header[6];
    header[0] = 0xB5;
    header[1] = 0x62;
    header[2] = cls;
    header[3] = id;
    header[4] = (uint8_t)(len & 0xFF);
    header[5] = (uint8_t)((len >> 8) & 0xFF);
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    ck_a += cls;
    ck_b += ck_a;
    ck_a += id;
    ck_b += ck_a;
    ck_a += header[4];
    ck_b += ck_a;
    ck_a += header[5];
    ck_b += ck_a;
    uart_write_bytes(s_uart, (const char *)header, sizeof(header));
    for (size_t i = 0; i < len; ++i) {
        ck_a += payload[i];
        ck_b += ck_a;
    }
    uart_write_bytes(s_uart, (const char *)payload, len);
    uint8_t checksum[2] = {ck_a, ck_b};
    uart_write_bytes(s_uart, (const char *)checksum, sizeof(checksum));
}

static bool wait_ubx_ack_locked(uint8_t target_cls, uint8_t target_id, TickType_t timeout) {
    TickType_t start = xTaskGetTickCount();
    uint8_t state = 0;
    uint8_t cls = 0;
    uint8_t id = 0;
    uint16_t length = 0;
    uint16_t consumed = 0;
    uint8_t payload[8];
    while ((xTaskGetTickCount() - start) < timeout) {
        uint8_t byte;
        int read = uart_read_bytes(s_uart, &byte, 1, pdMS_TO_TICKS(20));
        if (read <= 0) {
            continue;
        }
        switch (state) {
            case 0:
                state = (byte == 0xB5) ? 1 : 0;
                break;
            case 1:
                state = (byte == 0x62) ? 2 : 0;
                break;
            case 2:
                cls = byte;
                state = 3;
                break;
            case 3:
                id = byte;
                state = 4;
                break;
            case 4:
                length = byte;
                state = 5;
                break;
            case 5:
                length |= ((uint16_t)byte << 8);
                consumed = 0;
                state = 6;
                break;
            case 6:
                if (consumed < sizeof(payload)) {
                    payload[consumed] = byte;
                }
                consumed++;
                if (consumed >= length) {
                    state = 7;
                }
                break;
            case 7:
                state = 8;
                break;
            case 8:
                state = 0;
                if (cls == 0x05 && length == 2 && consumed >= 2) {
                    bool is_ack = (id == 0x01);
                    bool is_nak = (id == 0x00);
                    if ((is_ack || is_nak) && payload[0] == target_cls && payload[1] == target_id) {
                        ESP_LOGI(TAG, "GNSS UBX %s cls=0x%02x id=0x%02x", is_ack ? "ACK" : "NAK", target_cls,
                                 target_id);
                        return is_ack;
                    }
                }
                break;
            default:
                state = 0;
                break;
        }
    }
    ESP_LOGW(TAG, "GNSS UBX ACK timeout cls=0x%02x id=0x%02x", target_cls, target_id);
    return false;
}

static bool send_ubx_with_ack(uint8_t cls, uint8_t id, const uint8_t *payload, size_t len, TickType_t timeout) {
    if (!s_uart_lock) {
        return false;
    }
    if (xSemaphoreTake(s_uart_lock, pdMS_TO_TICKS(200)) != pdTRUE) {
        return false;
    }
    send_ubx_locked(cls, id, payload, len);
    bool ok = wait_ubx_ack_locked(cls, id, timeout);
    xSemaphoreGive(s_uart_lock);
    return ok;
}

void gnss_init(void) {
    gpio_config_t gpio = {
        .pin_bit_mask = BIT64(CONFIG_GNSS_LDO_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&gpio);
    gpio_set_level(CONFIG_GNSS_LDO_EN, 1);

    uart_config_t uart_cfg = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(s_uart, &uart_cfg);
    uart_set_pin(s_uart, CONFIG_GNSS_UART_TX, CONFIG_GNSS_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(s_uart, CONFIG_GNSS_UART_BUFFER, CONFIG_GNSS_UART_BUFFER, 0, NULL, 0);
    s_uart_lock = xSemaphoreCreateMutexStatic(&s_uart_lock_buffer);

    vTaskDelay(pdMS_TO_TICKS(200));
    send_command_with_ack("PMTK251,115200", "PMTK001,251,3", pdMS_TO_TICKS(500));
    vTaskDelay(pdMS_TO_TICKS(50));
    uart_set_baudrate(s_uart, CONFIG_GNSS_DEFAULT_BAUD);
    gnss_set_update_rate(CONFIG_GNSS_DEFAULT_RATE_HZ);

    ESP_LOGI(TAG, "GNSS initialized at %d baud %dHz", CONFIG_GNSS_DEFAULT_BAUD, CONFIG_GNSS_DEFAULT_RATE_HZ);
}

void gnss_poll(gnss_state_t *state) {
    if (!s_uart_lock) {
        return;
    }
    if (xSemaphoreTake(s_uart_lock, 0) != pdTRUE) {
        return;
    }
    uint8_t buf[64];
    int len = uart_read_bytes(s_uart, buf, sizeof(buf), 0);
    for (int i = 0; i < len; ++i) {
        consume_byte(state, buf[i]);
    }
    xSemaphoreGive(s_uart_lock);
}

bool gnss_set_update_rate(uint8_t hz) {
    if (hz == 0) {
        return false;
    }
    int interval_ms = 1000 / hz;
    char rate_cmd[32];
    snprintf(rate_cmd, sizeof(rate_cmd), "PMTK220,%d", interval_ms);
    bool ok = send_command_with_ack(rate_cmd, "PMTK001,220,3", pdMS_TO_TICKS(500));
    uint8_t payload[6] = {0};
    payload[0] = (uint8_t)(interval_ms & 0xFF);
    payload[1] = (uint8_t)((interval_ms >> 8) & 0xFF);
    payload[2] = 1;
    payload[3] = 0;
    payload[4] = 1;
    payload[5] = 0;
    bool ubx_ok = send_ubx_with_ack(0x06, 0x08, payload, sizeof(payload), pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "Set GNSS rate %uHz result=%d ubx=%d", hz, ok, ubx_ok);
    return ok && ubx_ok;
}

bool gnss_set_constellations(uint8_t mask) {
    int gps = (mask & SETTINGS_CONSTELLATION_GPS) ? 1 : 0;
    int glonass = (mask & SETTINGS_CONSTELLATION_GLONASS) ? 1 : 0;
    int galileo = (mask & SETTINGS_CONSTELLATION_GALILEO) ? 1 : 0;
    int beidou = (mask & SETTINGS_CONSTELLATION_BEIDOU) ? 1 : 0;
    char cmd[48];
    snprintf(cmd, sizeof(cmd), "PMTK353,%d,%d,%d,%d,0", gps, glonass, galileo, beidou);
    bool ok = send_command_with_ack(cmd, "PMTK001,353,3", pdMS_TO_TICKS(500));

    typedef struct __attribute__((packed)) {
        uint8_t gnssId;
        uint8_t resTrkCh;
        uint8_t maxTrkCh;
        uint8_t reserved1;
        uint32_t flags;
    } ubx_cfg_gnss_block_t;

    ubx_cfg_gnss_block_t blocks[4] = {
        {.gnssId = 0, .resTrkCh = 8, .maxTrkCh = 16},
        {.gnssId = 2, .resTrkCh = 8, .maxTrkCh = 16},
        {.gnssId = 3, .resTrkCh = 8, .maxTrkCh = 16},
        {.gnssId = 5, .resTrkCh = 8, .maxTrkCh = 16},
    };
    uint8_t enabled[4] = {gps, glonass, galileo, beidou};
    for (int i = 0; i < 4; ++i) {
        uint32_t flags = (uint32_t)(enabled[i] ? 1 : 0);
        flags |= (4u << 8);  // min channels
        flags |= (16u << 16); // max channels
        blocks[i].flags = flags;
    }
    uint8_t payload[4 + sizeof(blocks)] = {0};
    payload[0] = 0;   // version
    payload[1] = 32;  // numTrkChUse
    payload[2] = 32;  // numTrkChAvail
    payload[3] = 4;   // num blocks
    memcpy(&payload[4], blocks, sizeof(blocks));
    bool ubx_ok = send_ubx_with_ack(0x06, 0x3E, payload, sizeof(payload), pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "Set constellations mask=0x%02x result=%d ubx=%d", mask, ok, ubx_ok);
    return ok && ubx_ok;
}

const char *gnss_dynamic_mode_label(gnss_dynamic_mode_t mode) {
    switch (mode) {
        case GNSS_DYNAMIC_PEDESTRIAN:
            return "步行";
        case GNSS_DYNAMIC_AUTOMOTIVE:
            return "汽车";
        case GNSS_DYNAMIC_SEA:
            return "海上";
        case GNSS_DYNAMIC_AIRBORNE:
            return "航空";
        default:
            return "未知";
    }
}

bool gnss_set_dynamic_mode(gnss_dynamic_mode_t mode) {
    uint8_t payload[36] = {0};
    payload[0] = 0x05; // mask for dyn + fixMode
    payload[2] = (uint8_t)mode;
    payload[3] = 0x03; // auto 2D/3D
    bool ok = send_ubx_with_ack(0x06, 0x24, payload, sizeof(payload), pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "Set dynamic mode %d result=%d", mode, ok);
    return ok;
}
