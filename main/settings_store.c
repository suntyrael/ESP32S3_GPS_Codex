/*
 * settings_store.c - 系统设置与传感器校准参数 NVS Flash 持久化存储实现
 */
#include "settings_store.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "settings_store";

#define NVS_NS_SETTINGS "settings"
#define NVS_NS_CAL      "cal"
#define NVS_KEY_CALIB   "calib_blob"
#define NVS_KEY_CALIB_VER "calib_ver"

/* 校准数据版本：IMU 轴向映射变更后必须递增，使旧零偏符号失效 */
#define CALIB_VERSION   2u

static const char *s_keys[SETTINGS_STORE_COUNT] = {
    "func_mode",
    "theme",
    "gps_rate",
    "gnss_mode",
    "brightness",
    "auto_sleep",
    "rtc_sync",
    "alt_calib"
};

/* 默认配置项：默认 P-Box(0)、Sunlight(0)、10Hz(0)、GPS+GLO+BDS(0)、50%亮度(0)、3MIN(0)、ON(0)、ON(0) */
static int s_settings_cache[SETTINGS_STORE_COUNT] = { 0, 0, 0, 0, 0, 0, 0, 0 };

esp_err_t settings_store_init(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NS_SETTINGS, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open settings failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 从 NVS 逐项加载配置；如果不存在则写入默认值 */
    for (int i = 0; i < SETTINGS_STORE_COUNT; i++) {
        int32_t val = 0;
        err = nvs_get_i32(handle, s_keys[i], &val);
        if (err == ESP_OK) {
            s_settings_cache[i] = (int)val;
            ESP_LOGI(TAG, "Loaded %s = %d from NVS", s_keys[i], (int)val);
        } else {
            /* 写入默认值 */
            nvs_set_i32(handle, s_keys[i], (int32_t)s_settings_cache[i]);
            ESP_LOGI(TAG, "Initialized default %s = %d to NVS", s_keys[i], s_settings_cache[i]);
        }
    }

    nvs_commit(handle);
    nvs_close(handle);
    ESP_LOGI(TAG, "settings_store initialized ok");
    return ESP_OK;
}

int settings_store_get(int idx)
{
    if (idx >= 0 && idx < SETTINGS_STORE_COUNT) {
        return s_settings_cache[idx];
    }
    return 0;
}

void settings_store_set(int idx, int val)
{
    if (idx < 0 || idx >= SETTINGS_STORE_COUNT) {
        return;
    }
    if (s_settings_cache[idx] == val) {
        return;     /* 值未变化不重复写入，保护 Flash 寿命 */
    }

    s_settings_cache[idx] = val;

    nvs_handle_t handle;
    if (nvs_open(NVS_NS_SETTINGS, NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_i32(handle, s_keys[idx], (int32_t)val);
        nvs_commit(handle);
        nvs_close(handle);
        ESP_LOGI(TAG, "Saved setting %s -> %d to NVS", s_keys[idx], val);
    }
}

esp_err_t settings_store_save_all(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NS_SETTINGS, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    for (int i = 0; i < SETTINGS_STORE_COUNT; i++) {
        nvs_set_i32(handle, s_keys[i], (int32_t)s_settings_cache[i]);
    }
    nvs_commit(handle);
    nvs_close(handle);
    return ESP_OK;
}

/* ==================== 传感器校准参数持久化 ==================== */
esp_err_t calib_store_load(sensor_calib_data_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));
    out->mag_scale[0] = 1.0f;
    out->mag_scale[1] = 1.0f;
    out->mag_scale[2] = 1.0f;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NS_CAL, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return err;
    }

    size_t req_size = sizeof(*out);
    err = nvs_get_blob(handle, NVS_KEY_CALIB, out, &req_size);

    if (err == ESP_OK) {
        /* 版本校验：轴向映射变更后，旧 IMU 零偏的符号已失效，仅作废 IMU 部分，保留地磁校准 */
        uint32_t ver = 0;
        if (nvs_get_u32(handle, NVS_KEY_CALIB_VER, &ver) != ESP_OK || ver != CALIB_VERSION) {
            out->imu_calibrated = false;
            memset(out->gyro_bias_mdps, 0, sizeof(out->gyro_bias_mdps));
            memset(out->acc_bias_mg, 0, sizeof(out->acc_bias_mg));
            ESP_LOGW(TAG, "Calib version mismatch (stored=%u, expected=%u): IMU calibration invalidated, mag kept",
                     (unsigned)ver, (unsigned)CALIB_VERSION);
        }
    }
    nvs_close(handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Loaded calib from NVS: IMU_cal=%d, MAG_cal=%d",
                 (int)out->imu_calibrated, (int)out->mag_calibrated);
        ESP_LOGI(TAG, "IMU bias: gyro=(%+.1f,%+.1f,%+.1f) mdps acc=(%+.0f,%+.0f,%+.0f) mg",
                 (double)out->gyro_bias_mdps[0], (double)out->gyro_bias_mdps[1], (double)out->gyro_bias_mdps[2],
                 (double)out->acc_bias_mg[0], (double)out->acc_bias_mg[1], (double)out->acc_bias_mg[2]);
        ESP_LOGI(TAG, "MAG bias: (%+.1f,%+.1f,%+.1f) mG  scale=(%.3f,%.3f,%.3f)",
                 (double)out->mag_bias_mgauss[0], (double)out->mag_bias_mgauss[1], (double)out->mag_bias_mgauss[2],
                 (double)out->mag_scale[0], (double)out->mag_scale[1], (double)out->mag_scale[2]);
    }
    return err;
}

esp_err_t calib_store_save(const sensor_calib_data_t *in)
{
    if (in == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NS_CAL, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(handle, NVS_KEY_CALIB, in, sizeof(*in));
    if (err == ESP_OK) {
        nvs_set_u32(handle, NVS_KEY_CALIB_VER, CALIB_VERSION);
        nvs_commit(handle);
        ESP_LOGI(TAG, "Saved calib data to NVS successfully");
    }
    nvs_close(handle);
    return err;
}
