#include "settings_store.h"

#include <string.h>

#include "esp_log.h"
#include "nvs.h"

#include "config.h"

static const char *TAG = "settings";
static nvs_handle_t s_nvs_handle;
static persisted_settings_t s_settings = {
    .gnss_rate_hz = CONFIG_GNSS_DEFAULT_RATE_HZ,
    .gnss_constellation_mask = SETTINGS_CONSTELLATION_GPS | SETTINGS_CONSTELLATION_GLONASS |
                               SETTINGS_CONSTELLATION_GALILEO | SETTINGS_CONSTELLATION_BEIDOU,
    .gnss_dynamic_mode = GNSS_DYNAMIC_AUTOMOTIVE,
    .pbox_start_accel_g = CONFIG_PBOX_START_ACCEL_G,
};

static void persist_locked(void) {
    if (!s_nvs_handle) {
        return;
    }
    esp_err_t err = nvs_set_blob(s_nvs_handle, "settings", &s_settings, sizeof(s_settings));
    if (err == ESP_OK) {
        nvs_commit(s_nvs_handle);
    } else {
        ESP_LOGE(TAG, "Failed to store settings (%d)", err);
    }
}

void settings_store_init(void) {
    esp_err_t err = nvs_open("cfg", NVS_READWRITE, &s_nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed (%d)", err);
        s_nvs_handle = 0;
        return;
    }
    size_t size = sizeof(s_settings);
    err = nvs_get_blob(s_nvs_handle, "settings", &s_settings, &size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Settings not found, using defaults");
        persist_locked();
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read settings (%d)", err);
    } else if (size < sizeof(s_settings)) {
        ESP_LOGI(TAG, "Expanding settings blob (was %u bytes)", (unsigned int)size);
        if (s_settings.gnss_dynamic_mode > GNSS_DYNAMIC_AIRBORNE) {
            s_settings.gnss_dynamic_mode = GNSS_DYNAMIC_AUTOMOTIVE;
        }
        persist_locked();
    }
}

const persisted_settings_t *settings_store_get(void) {
    return &s_settings;
}

bool settings_store_set_gnss_rate(uint8_t hz) {
    if (hz == 0) {
        return false;
    }
    if (s_settings.gnss_rate_hz == hz) {
        return true;
    }
    s_settings.gnss_rate_hz = hz;
    persist_locked();
    ESP_LOGI(TAG, "GNSS rate saved: %uHz", hz);
    return true;
}

bool settings_store_set_constellation_mask(uint8_t mask) {
    if (mask == 0) {
        return false;
    }
    if (s_settings.gnss_constellation_mask == mask) {
        return true;
    }
    s_settings.gnss_constellation_mask = mask;
    persist_locked();
    ESP_LOGI(TAG, "Constellation mask saved: 0x%02x", mask);
    return true;
}

bool settings_store_set_dynamic_mode(gnss_dynamic_mode_t mode) {
    if (s_settings.gnss_dynamic_mode == mode) {
        return true;
    }
    s_settings.gnss_dynamic_mode = mode;
    persist_locked();
    ESP_LOGI(TAG, "GNSS dynamic mode saved: %d", mode);
    return true;
}

bool settings_store_set_pbox_threshold(float accel_g) {
    if (accel_g <= 0.0f) {
        return false;
    }
    if (s_settings.pbox_start_accel_g == accel_g) {
        return true;
    }
    s_settings.pbox_start_accel_g = accel_g;
    persist_locked();
    ESP_LOGI(TAG, "P-Box threshold saved: %.2fG", accel_g);
    return true;
}
