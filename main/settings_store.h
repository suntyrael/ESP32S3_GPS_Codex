#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t gnss_rate_hz;
    uint8_t gnss_constellation_mask;
    float pbox_start_accel_g;
} persisted_settings_t;

// 星座掩码定义
#define SETTINGS_CONSTELLATION_GPS      (1 << 0)
#define SETTINGS_CONSTELLATION_GLONASS  (1 << 1)
#define SETTINGS_CONSTELLATION_GALILEO  (1 << 2)
#define SETTINGS_CONSTELLATION_BEIDOU   (1 << 3)

void settings_store_init(void);
const persisted_settings_t *settings_store_get(void);
bool settings_store_set_gnss_rate(uint8_t hz);
bool settings_store_set_constellation_mask(uint8_t mask);
bool settings_store_set_pbox_threshold(float accel_g);

#ifdef __cplusplus
}
#endif
