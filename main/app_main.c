#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lvgl.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// --- 业务相关头文件 ---
#include "config.h"
#include "diagnostics.h"
#include "gpx_logger.h"
#include "gnss.h"
#include "gnss_types.h"
#include "input_manager.h"
#include "sensors.h"
#include "settings_store.h"
#include "ui/ui_bike_computer.h"
#include "ui/ui_gnss_info.h"
#include "ui/ui_gps_logger.h"
#include "ui/ui_pbox.h"
#include "ui/ui_settings.h"
#include "ui/ui_state_bar.h"

#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "app";

// --- UI 模式与系统上下文类型定义 ---
typedef enum {
    MODE_BIKE = 0,
    MODE_GPS_LOGGER,
    MODE_PBOX,
    MODE_GNSS_INFO,
    MODE_SETTINGS,
    MODE_COUNT
} ui_mode_t;

typedef struct {
    ui_mode_t mode;
    float      ride_distance_km;
    uint32_t   ride_time_s;
    float      track_distance_km;
    uint32_t   track_time_s;
    pbox_status_t        pbox_status;
    float                pbox_target_kmh;
    float                pbox_elapsed_s;
    TickType_t           pbox_start_tick;
    settings_option_t    settings_selected;
    uint8_t              gnss_rate_hz;
    uint8_t              gnss_constellation_mask;
    gnss_dynamic_mode_t  gnss_dynamic_mode;
    float                pbox_start_accel_g;
} system_context_t;

static system_context_t s_ctx = {
    .mode                   = MODE_BIKE,
    .pbox_status            = PBOX_STATUS_READY,
    .pbox_target_kmh        = CONFIG_PBOX_TARGET_SPEED_KMH,
    .gnss_rate_hz           = CONFIG_GNSS_DEFAULT_RATE_HZ,
    .gnss_constellation_mask = SETTINGS_CONSTELLATION_GPS |
                               SETTINGS_CONSTELLATION_GLONASS |
                               SETTINGS_CONSTELLATION_GALILEO |
                               SETTINGS_CONSTELLATION_BEIDOU,
    .gnss_dynamic_mode      = GNSS_DYNAMIC_AUTOMOTIVE,
    .pbox_start_accel_g     = CONFIG_PBOX_START_ACCEL_G,
};

// 保护 s_ctx 的互斥锁
static SemaphoreHandle_t s_ctx_lock;

// UI 根对象
static lv_obj_t *s_status_bar = NULL;
static lv_obj_t *s_screens[MODE_COUNT] = {0};

static const uint8_t kGnssRates[] = {1, 5, 10, 25};
static const size_t kGnssRatesCount = sizeof(kGnssRates) / sizeof(kGnssRates[0]);

static const gnss_dynamic_mode_t kDynamicModes[] = {
    GNSS_DYNAMIC_PEDESTRIAN,
    GNSS_DYNAMIC_AUTOMOTIVE,
    GNSS_DYNAMIC_SEA,
    GNSS_DYNAMIC_AIRBORNE,
};

typedef struct {
    uint8_t     mask;
    const char *label;
} constellation_option_t;

static const constellation_option_t kConstellations[] = {
    {SETTINGS_CONSTELLATION_GPS | SETTINGS_CONSTELLATION_GLONASS, "GPS+GLO"},
    {SETTINGS_CONSTELLATION_GPS | SETTINGS_CONSTELLATION_GALILEO, "GPS+GAL"},
    {SETTINGS_CONSTELLATION_GPS | SETTINGS_CONSTELLATION_BEIDOU, "GPS+BD"},
    {SETTINGS_CONSTELLATION_GPS | SETTINGS_CONSTELLATION_GLONASS | SETTINGS_CONSTELLATION_BEIDOU, "GPS+GLO+BD"},
};

static const float kPboxThresholds[] = {0.10f, 0.15f, 0.20f, 0.25f, 0.30f};

// ---------------- 辅助函数 ----------------

static const char *constellation_label(uint8_t mask) {
    for (size_t i = 0; i < sizeof(kConstellations) / sizeof(kConstellations[0]); ++i) {
        if (kConstellations[i].mask == mask) {
            return kConstellations[i].label;
        }
    }
    static char label[16];
    snprintf(label, sizeof(label), "0x%02x", mask);
    return label;
}

static uint8_t next_gnss_rate(uint8_t current) {
    for (size_t i = 0; i < kGnssRatesCount; ++i) {
        if (kGnssRates[i] == current) {
            return kGnssRates[(i + 1) % kGnssRatesCount];
        }
    }
    return kGnssRates[0];
}

static uint8_t next_constellation(uint8_t current) {
    size_t count = sizeof(kConstellations) / sizeof(kConstellations[0]);
    for (size_t i = 0; i < count; ++i) {
        if (kConstellations[i].mask == current) {
            return kConstellations[(i + 1) % count].mask;
        }
    }
    return kConstellations[0].mask;
}

static float next_pbox_threshold(float current) {
    size_t count = sizeof(kPboxThresholds) / sizeof(kPboxThresholds[0]);
    for (size_t i = 0; i < count; ++i) {
        if (fabsf(kPboxThresholds[i] - current) < 0.001f) {
            return kPboxThresholds[(i + 1) % count];
        }
    }
    return kPboxThresholds[0];
}

static gnss_dynamic_mode_t next_dynamic_mode(gnss_dynamic_mode_t current) {
    size_t count = sizeof(kDynamicModes) / sizeof(kDynamicModes[0]);
    for (size_t i = 0; i < count; ++i) {
        if (kDynamicModes[i] == current) {
            return kDynamicModes[(i + 1) % count];
        }
    }
    return kDynamicModes[0];
}

static const char *pbox_status_label(pbox_status_t status) {
    switch (status) {
        case PBOX_STATUS_READY:    return "ready";
        case PBOX_STATUS_ARMED:    return "armed";
        case PBOX_STATUS_RUNNING:  return "running";
        case PBOX_STATUS_FINISHED: return "finished";
        default:                   return "unknown";
    }
}

static const char *mode_label(ui_mode_t mode) {
    switch (mode) {
        case MODE_BIKE:        return "bike";
        case MODE_GPS_LOGGER:  return "logger";
        case MODE_PBOX:        return "pbox";
        case MODE_GNSS_INFO:   return "gnss";
        case MODE_SETTINGS:    return "settings";
        default:               return "unknown";
    }
}

// ---------------- 输入 & 状态更新 ----------------

static void cycle_mode(int direction) {
    if (xSemaphoreTake(s_ctx_lock, pdMS_TO_TICKS(10)) != pdTRUE) return;

    if (s_ctx.mode == MODE_SETTINGS) {
        if (direction > 0) {
            s_ctx.settings_selected = (s_ctx.settings_selected + 1) % SETTINGS_OPTION_COUNT;
        } else {
            s_ctx.settings_selected = (s_ctx.settings_selected + SETTINGS_OPTION_COUNT - 1) % SETTINGS_OPTION_COUNT;
        }
        xSemaphoreGive(s_ctx_lock);
        return;
    }

    int next = (int)s_ctx.mode + direction;
    if (next < 0) {
        next = MODE_COUNT - 1;
    } else if (next >= MODE_COUNT) {
        next = 0;
    }
    s_ctx.mode = (ui_mode_t)next;

    xSemaphoreGive(s_ctx_lock);
}

static void apply_settings_action(void) {
    // 1. 锁内读取当前值
    settings_option_t   selected_option;
    uint8_t             current_rate;
    gnss_dynamic_mode_t current_mode;
    uint8_t             current_mask;
    float               current_thresh;

    if (xSemaphoreTake(s_ctx_lock, pdMS_TO_TICKS(10)) != pdTRUE) return;
    selected_option = s_ctx.settings_selected;
    current_rate    = s_ctx.gnss_rate_hz;
    current_mode    = s_ctx.gnss_dynamic_mode;
    current_mask    = s_ctx.gnss_constellation_mask;
    current_thresh  = s_ctx.pbox_start_accel_g;
    xSemaphoreGive(s_ctx_lock);

    // 2. 锁外做 I/O 操作
    switch (selected_option) {
        case SETTINGS_OPTION_GNSS_RATE: {
            uint8_t next = next_gnss_rate(current_rate);
            bool ok = gnss_set_update_rate(next);
            diagnostics_trigger_event(ok ? "GNSS_RATE_OK" : "GNSS_RATE_FAIL", next);
            if (ok) {
                if (xSemaphoreTake(s_ctx_lock, pdMS_TO_TICKS(10)) == pdTRUE) {
                    s_ctx.gnss_rate_hz = next;
                    xSemaphoreGive(s_ctx_lock);
                }
                settings_store_set_gnss_rate(next);
            }
            break;
        }
        case SETTINGS_OPTION_GNSS_DYNAMIC: {
            gnss_dynamic_mode_t next = next_dynamic_mode(current_mode);
            bool ok = gnss_set_dynamic_mode(next);
            diagnostics_trigger_event(ok ? "GNSS_MODE_OK" : "GNSS_MODE_FAIL", next);
            if (ok) {
                if (xSemaphoreTake(s_ctx_lock, pdMS_TO_TICKS(10)) == pdTRUE) {
                    s_ctx.gnss_dynamic_mode = next;
                    xSemaphoreGive(s_ctx_lock);
                }
                settings_store_set_dynamic_mode(next);
            }
            break;
        }
        case SETTINGS_OPTION_CONSTELLATION: {
            uint8_t next = next_constellation(current_mask);
            bool ok = gnss_set_constellations(next);
            diagnostics_trigger_event(ok ? "GNSS_CONST_OK" : "GNSS_CONST_FAIL", next);
            if (ok) {
                if (xSemaphoreTake(s_ctx_lock, pdMS_TO_TICKS(10)) == pdTRUE) {
                    s_ctx.gnss_constellation_mask = next;
                    xSemaphoreGive(s_ctx_lock);
                }
                settings_store_set_constellation_mask(next);
            }
            break;
        }
        case SETTINGS_OPTION_PBOX_THRESHOLD: {
            float next = next_pbox_threshold(current_thresh);
            if (xSemaphoreTake(s_ctx_lock, pdMS_TO_TICKS(10)) == pdTRUE) {
                s_ctx.pbox_start_accel_g = next;
                xSemaphoreGive(s_ctx_lock);
            }
            settings_store_set_pbox_threshold(next);
            diagnostics_trigger_event("PBOX_THRESH", (uint32_t)(next * 1000));
            break;
        }
        case SETTINGS_OPTION_IMU_CAL: {
            bool started = sensors_start_calibration(SENSORS_CALIBRATION_IMU);
            diagnostics_trigger_event(started ? "IMU_CAL_START" : "IMU_CAL_BUSY", 0);
            break;
        }
        case SETTINGS_OPTION_MAG_CAL: {
            bool started = sensors_start_calibration(SENSORS_CALIBRATION_MAG);
            diagnostics_trigger_event(started ? "MAG_CAL_START" : "MAG_CAL_BUSY", 0);
            break;
        }
        default:
            diagnostics_trigger_event("SETTINGS_SELECT", 0);
            break;
    }
}

static void handle_button_short(void) {
    if (xSemaphoreTake(s_ctx_lock, pdMS_TO_TICKS(10)) != pdTRUE) return;

    if (s_ctx.mode == MODE_SETTINGS) {
        xSemaphoreGive(s_ctx_lock);
        apply_settings_action();
        return;
    }

    if (s_ctx.mode == MODE_PBOX) {
        if (s_ctx.pbox_status == PBOX_STATUS_READY) {
            s_ctx.pbox_status = PBOX_STATUS_ARMED;
        } else if (s_ctx.pbox_status == PBOX_STATUS_FINISHED) {
            s_ctx.pbox_status = PBOX_STATUS_READY;
            s_ctx.pbox_elapsed_s = 0.0f;
        }
    }

    xSemaphoreGive(s_ctx_lock);
}

static void handle_button_medium(void) {
    if (gpx_logger_get_state() == GPX_LOGGER_STATE_RECORDING) {
        gpx_logger_stop();
    } else {
        gpx_logger_start();
    }
}

static void handle_button_long(void) {
    if (xSemaphoreTake(s_ctx_lock, pdMS_TO_TICKS(10)) != pdTRUE) return;

    if (s_ctx.mode == MODE_SETTINGS) {
        s_ctx.mode = MODE_BIKE;
    } else {
        s_ctx.mode = MODE_SETTINGS;
    }

    xSemaphoreGive(s_ctx_lock);
}

// ---------------- 传感器与 PBOX 逻辑（需持锁调用） ----------------

static void locked_update_pbox_logic(const sensors_state_t *state, float delta_s) {
    switch (s_ctx.pbox_status) {
        case PBOX_STATUS_ARMED:
            if (state->gnss.speed_kmh < CONFIG_PBOX_START_SPEED_KMH &&
                state->imu.linear_accel_g.x > s_ctx.pbox_start_accel_g) {
                s_ctx.pbox_status      = PBOX_STATUS_RUNNING;
                s_ctx.pbox_start_tick  = xTaskGetTickCount();
                s_ctx.pbox_elapsed_s   = 0.0f;
            }
            break;
        case PBOX_STATUS_RUNNING:
            s_ctx.pbox_elapsed_s += delta_s;
            if (state->gnss.speed_kmh >= s_ctx.pbox_target_kmh) {
                s_ctx.pbox_status = PBOX_STATUS_FINISHED;
            }
            break;
        default:
            break;
    }
}

static void locked_update_trip_metrics(const sensors_state_t *state, float delta_s) {
    float speed_mps = state->gnss.speed_kmh / 3.6f;
    s_ctx.ride_distance_km += (speed_mps * delta_s) / 1000.0f;
    s_ctx.ride_time_s      += (uint32_t)delta_s;
    if (gpx_logger_get_state() == GPX_LOGGER_STATE_RECORDING) {
        s_ctx.track_distance_km += (speed_mps * delta_s) / 1000.0f;
        s_ctx.track_time_s      += (uint32_t)delta_s;
    }
}

// ---------------- 各个任务实现 ----------------

static void sensor_task(void *arg) {
    (void)arg;
    TickType_t last_tick = xTaskGetTickCount();

    while (1) {
        sensors_update();
        sensors_state_t state;
        sensors_get_state(&state);

        TickType_t now = xTaskGetTickCount();
        TickType_t delta_ticks = now - last_tick;

        if (delta_ticks > 0) {
            float delta_s = (float)delta_ticks / (float)configTICK_RATE_HZ;
            gpx_sample_metadata_t meta;

            if (xSemaphoreTake(s_ctx_lock, pdMS_TO_TICKS(10)) == pdTRUE) {
                locked_update_trip_metrics(&state, delta_s);
                locked_update_pbox_logic(&state, delta_s);

                meta = (gpx_sample_metadata_t){
                    .timestamp_utc      = state.gnss.timestamp_utc,
                    .battery_percent    = state.power.battery_percent,
                    .battery_voltage_v  = state.power.battery_voltage_v,
                    .ride_distance_km   = s_ctx.ride_distance_km,
                    .track_distance_km  = s_ctx.track_distance_km,
                    .pbox_elapsed_s     = s_ctx.pbox_elapsed_s,
                };
                snprintf(meta.mode_label, sizeof(meta.mode_label), "%s", mode_label(s_ctx.mode));
                const char *context = (s_ctx.mode == MODE_PBOX)
                                      ? pbox_status_label(s_ctx.pbox_status)
                                      : "ride";
                snprintf(meta.context_label, sizeof(meta.context_label), "%s", context);

                xSemaphoreGive(s_ctx_lock);

                gpx_logger_push_sample(&state, &meta);
            }
        }

        last_tick = now;
        vTaskDelay(pdMS_TO_TICKS(40)); // 25Hz
    }
}

static void diagnostic_task(void *arg) {
    (void)arg;
    TickType_t boot_deadline = pdMS_TO_TICKS(CONFIG_DIAG_BOOT_DURATION_MS);
    TickType_t start = xTaskGetTickCount();

    while (1) {
        sensors_state_t state;
        sensors_get_state(&state);

        TickType_t now = xTaskGetTickCount();
        TickType_t uptime = now - start;

        if (uptime < boot_deadline) {
            diagnostics_report_boot(&state, (uptime * 1000) / configTICK_RATE_HZ);
            vTaskDelay(pdMS_TO_TICKS(CONFIG_DIAG_BOOT_INTERVAL_MS));
        } else {
            diagnostics_report_heartbeat(&state, (uptime * 1000) / configTICK_RATE_HZ);
            vTaskDelay(pdMS_TO_TICKS(CONFIG_DIAG_HEARTBEAT_MS));
        }
    }
}

static void input_task(void *arg) {
    (void)arg;
    input_event_t event;

    while (1) {
        if (input_manager_get_event(&event, portMAX_DELAY)) {
            switch (event.type) {
                case INPUT_EVENT_ENCODER_LEFT:
                    cycle_mode(-1);
                    break;
                case INPUT_EVENT_ENCODER_RIGHT:
                    cycle_mode(1);
                    break;
                case INPUT_EVENT_BUTTON_SHORT:
                    handle_button_short();
                    break;
                case INPUT_EVENT_BUTTON_MEDIUM:
                    handle_button_medium();
                    break;
                case INPUT_EVENT_BUTTON_LONG:
                    handle_button_long();
                    break;
                case INPUT_EVENT_BUTTON_DOUBLE:
                case INPUT_EVENT_NONE:
                default:
                    break;
            }
        }
    }
}

// ---------------- LVGL 场景 & UI 刷新 ----------------

// 仅负责搭建 UI，不再调用 lv_init()
static void init_lvgl_scene(void) {
    lv_obj_t *s_root = lv_scr_act();

    if (s_root == NULL) {
        ESP_LOGE(TAG, "lv_scr_act() returned NULL. Is LVGL display driver initialized?");
        return;
    }

    ui_apply_theme(s_root);
    lv_obj_set_size(s_root, UI_SCREEN_WIDTH, UI_SCREEN_HEIGHT);
    lv_obj_set_flex_flow(s_root, LV_FLEX_FLOW_COLUMN);

    s_status_bar = ui_state_bar_create(s_root);
    s_screens[MODE_BIKE]       = ui_bike_computer_create(s_root);
    s_screens[MODE_GPS_LOGGER] = ui_gps_logger_create(s_root);
    s_screens[MODE_PBOX]       = ui_pbox_create(s_root);
    s_screens[MODE_GNSS_INFO]  = ui_gnss_info_create(s_root);
    s_screens[MODE_SETTINGS]   = ui_settings_create(s_root);
}

static void refresh_ui(const sensors_state_t *state) {
    // UI 可能初始化失败，这里要防御一下
    if (s_status_bar == NULL) {
        // 屏蔽刷屏日志，只在需要时打开
        // ESP_LOGW(TAG, "UI not initialized, skip refresh");
        return;
    }
    for (int i = 0; i < MODE_COUNT; ++i) {
        if (s_screens[i] == NULL) {
            // ESP_LOGW(TAG, "Screen %d not initialized, skip refresh", i);
            return;
        }
    }

    if (xSemaphoreTake(s_ctx_lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }

    ui_telemetry_t telemetry = {
        .sensors      = *state,
        .logger_state = gpx_logger_get_state(),
    };

    ui_state_bar_update(s_status_bar, &telemetry);

    for (int i = 0; i < MODE_COUNT; ++i) {
        lv_obj_add_flag(s_screens[i], LV_OBJ_FLAG_HIDDEN);
    }

    lv_obj_clear_flag(s_screens[s_ctx.mode], LV_OBJ_FLAG_HIDDEN);

    switch (s_ctx.mode) {
        case MODE_BIKE:
            ui_bike_computer_update(
                s_screens[MODE_BIKE],
                &telemetry,
                s_ctx.ride_distance_km,
                s_ctx.ride_time_s);
            break;
        case MODE_GPS_LOGGER:
            ui_gps_logger_update(
                s_screens[MODE_GPS_LOGGER],
                &telemetry,
                s_ctx.track_distance_km,
                s_ctx.track_time_s);
            break;
        case MODE_PBOX:
            ui_pbox_update(
                s_screens[MODE_PBOX],
                &telemetry,
                s_ctx.pbox_status,
                s_ctx.pbox_target_kmh,
                s_ctx.pbox_elapsed_s);
            break;
        case MODE_GNSS_INFO:
            ui_gnss_info_update(s_screens[MODE_GNSS_INFO], &telemetry);
            break;
        case MODE_SETTINGS: {
            sensors_calibration_status_t cal_status;
            sensors_get_calibration_status(&cal_status);

            char imu_hint[48];
            char mag_hint[48];

            snprintf(imu_hint, sizeof(imu_hint), "%s (%s)", "IMU 校准", "按下开始");
            snprintf(mag_hint, sizeof(mag_hint), "%s (%s)", "磁力计校准", "按下开始");

            if (cal_status.active_type == SENSORS_CALIBRATION_IMU) {
                const char *msg = cal_status.message[0] ? cal_status.message : "";
                snprintf(imu_hint, sizeof(imu_hint),
                         "IMU 校准 (%.24s %.0f%%)", msg, cal_status.progress * 100.0f);
            } else if (cal_status.active_type == SENSORS_CALIBRATION_MAG) {
                const char *msg = cal_status.message[0] ? cal_status.message : "";
                snprintf(mag_hint, sizeof(mag_hint),
                         "磁力计校准 (%.24s %.0f%%)", msg, cal_status.progress * 100.0f);
            }

            settings_view_model_t model = {
                .selected            = s_ctx.settings_selected,
                .gnss_rate_hz        = s_ctx.gnss_rate_hz,
                .constellation_label = constellation_label(s_ctx.gnss_constellation_mask),
                .dynamic_label       = gnss_dynamic_mode_label(s_ctx.gnss_dynamic_mode),
                .pbox_threshold_g    = s_ctx.pbox_start_accel_g,
            };

            strncpy(model.imu_status, imu_hint, sizeof(model.imu_status));
            model.imu_status[sizeof(model.imu_status) - 1] = '\0';
            strncpy(model.mag_status, mag_hint, sizeof(model.mag_status));
            model.mag_status[sizeof(model.mag_status) - 1] = '\0';

            ui_settings_update(s_screens[MODE_SETTINGS], &model);
            break;
        }
        default:
            break;
    }

    xSemaphoreGive(s_ctx_lock);
}

static void ui_task(void *arg) {
    (void)arg;

    init_lvgl_scene();  // 这里只搭场景，不再调用 lv_init()

    while (1) {
        sensors_state_t state;
        sensors_get_state(&state);
        refresh_ui(&state);
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz UI 刷新
    }
}

// ---------------- app_main ----------------

void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // 创建互斥锁
    s_ctx_lock = xSemaphoreCreateMutex();
    if (s_ctx_lock == NULL) {
        ESP_LOGE(TAG, "Failed to create s_ctx_lock! Aborting.");
        abort();
    }

    // 初始化 LVGL 核心
    lv_init();
    // ⚠️ 如果你有 lv_port_disp_init / lv_port_indev_init 之类的函数
    // 请在这里调用它们来初始化显示 & 输入驱动
    // 例如：
    // lv_port_disp_init();
    // lv_port_indev_init();

    settings_store_init();
    const persisted_settings_t *stored = settings_store_get();
    if (stored) {
        s_ctx.gnss_rate_hz          = stored->gnss_rate_hz;
        s_ctx.gnss_constellation_mask = stored->gnss_constellation_mask;
        s_ctx.gnss_dynamic_mode     = stored->gnss_dynamic_mode;
        s_ctx.pbox_start_accel_g    = stored->pbox_start_accel_g;
    }

    sensors_init();
    diagnostics_init();
    input_manager_init();
    gpx_logger_init();

    // GNSS 初始配置
    gnss_set_update_rate(s_ctx.gnss_rate_hz);
    gnss_set_constellations(s_ctx.gnss_constellation_mask);
    gnss_set_dynamic_mode(s_ctx.gnss_dynamic_mode);

    xTaskCreate(sensor_task,     "sensor",    CONFIG_TASK_STACK_DEFAULT,
                NULL, CONFIG_TASK_PRIO_SENSOR,  NULL);
    xTaskCreate(diagnostic_task, "diag",      CONFIG_TASK_STACK_DEFAULT,
                NULL, CONFIG_TASK_PRIO_DIAG,    NULL);
    xTaskCreate(input_task,      "input_ctl", CONFIG_TASK_STACK_DEFAULT,
                NULL, CONFIG_TASK_PRIO_INPUT,   NULL);
    xTaskCreate(ui_task,         "ui",        CONFIG_TASK_STACK_DEFAULT * 2,
                NULL, CONFIG_TASK_PRIO_UI,      NULL);

    ESP_LOGI(TAG, "System initialized");
}
