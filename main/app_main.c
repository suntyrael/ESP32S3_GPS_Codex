#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"

#include "config.h"
#include "diagnostics.h"
#include "gpx_logger.h"
#include "input_manager.h"
#include "sensors.h"
#include "ui/ui_bike_computer.h"
#include "ui/ui_gnss_info.h"
#include "ui/ui_gps_logger.h"
#include "ui/ui_pbox.h"
#include "ui/ui_settings.h"
#include "ui/ui_state_bar.h"

#include "esp_log.h"

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
    float ride_distance_km;
    uint32_t ride_time_s;
    float track_distance_km;
    uint32_t track_time_s;
    pbox_status_t pbox_status;
    float pbox_target_kmh;
    float pbox_elapsed_s;
    TickType_t pbox_start_tick;
    settings_option_t settings_selected;
    uint8_t gnss_rate_hz;
} system_context_t;

static const char *TAG = "app";
static system_context_t s_ctx = {
    .mode = MODE_BIKE,
    .pbox_status = PBOX_STATUS_READY,
    .pbox_target_kmh = CONFIG_PBOX_TARGET_SPEED_KMH,
    .gnss_rate_hz = CONFIG_GNSS_DEFAULT_RATE_HZ,
};

static lv_obj_t *s_root;
static lv_obj_t *s_status_bar;
static lv_obj_t *s_screens[MODE_COUNT];

static void cycle_mode(int direction) {
    if (s_ctx.mode == MODE_SETTINGS) {
        if (direction > 0) {
            s_ctx.settings_selected = (s_ctx.settings_selected + 1) % SETTINGS_OPTION_COUNT;
        } else {
            s_ctx.settings_selected = (s_ctx.settings_selected + SETTINGS_OPTION_COUNT - 1) % SETTINGS_OPTION_COUNT;
        }
        return;
    }
    int next = (int)s_ctx.mode + direction;
    if (next < 0) {
        next = MODE_COUNT - 1;
    }
    if (next >= MODE_COUNT) {
        next = 0;
    }
    s_ctx.mode = (ui_mode_t)next;
}

static void handle_button_short(void) {
    if (s_ctx.mode == MODE_PBOX) {
        if (s_ctx.pbox_status == PBOX_STATUS_READY) {
            s_ctx.pbox_status = PBOX_STATUS_ARMED;
        } else if (s_ctx.pbox_status == PBOX_STATUS_FINISHED) {
            s_ctx.pbox_status = PBOX_STATUS_READY;
            s_ctx.pbox_elapsed_s = 0.0f;
        }
    }
}

static void handle_button_medium(void) {
    if (gpx_logger_get_state() == GPX_LOGGER_STATE_RECORDING) {
        gpx_logger_stop();
    } else {
        gpx_logger_start();
    }
}

static void handle_button_long(void) {
    if (s_ctx.mode == MODE_SETTINGS) {
        s_ctx.mode = MODE_BIKE;
    } else {
        s_ctx.mode = MODE_SETTINGS;
    }
}

static void update_pbox_logic(const sensors_state_t *state, float delta_s) {
    switch (s_ctx.pbox_status) {
        case PBOX_STATUS_ARMED:
            if (state->gnss.speed_kmh < CONFIG_PBOX_START_SPEED_KMH &&
                state->imu.linear_accel_g.x > CONFIG_PBOX_START_ACCEL_G) {
                s_ctx.pbox_status = PBOX_STATUS_RUNNING;
                s_ctx.pbox_start_tick = xTaskGetTickCount();
                s_ctx.pbox_elapsed_s = 0.0f;
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

static void update_trip_metrics(const sensors_state_t *state, float delta_s) {
    float speed_mps = state->gnss.speed_kmh / 3.6f;
    s_ctx.ride_distance_km += (speed_mps * delta_s) / 1000.0f;
    s_ctx.ride_time_s += (uint32_t)delta_s;
    if (gpx_logger_get_state() == GPX_LOGGER_STATE_RECORDING) {
        s_ctx.track_distance_km += (speed_mps * delta_s) / 1000.0f;
        s_ctx.track_time_s += (uint32_t)delta_s;
    }
}

static void sensor_task(void *arg) {
    (void)arg;
    TickType_t last_tick = xTaskGetTickCount();
    while (1) {
        sensors_update();
        sensors_state_t state;
        sensors_get_state(&state);
        TickType_t now = xTaskGetTickCount();
        TickType_t delta_ticks = now - last_tick;
        last_tick = now;
        float delta_s = (float)delta_ticks / configTICK_RATE_HZ;
        update_trip_metrics(&state, delta_s);
        update_pbox_logic(&state, delta_s);
        gpx_logger_push_sample(&state);
        vTaskDelay(pdMS_TO_TICKS(200));
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

static void init_lvgl_scene(void) {
    lv_init();
    s_root = lv_scr_act();
    ui_apply_theme(s_root);
    lv_obj_set_size(s_root, UI_SCREEN_WIDTH, UI_SCREEN_HEIGHT);
    lv_obj_set_flex_flow(s_root, LV_FLEX_FLOW_COLUMN);
    s_status_bar = ui_state_bar_create(s_root);
    s_screens[MODE_BIKE] = ui_bike_computer_create(s_root);
    s_screens[MODE_GPS_LOGGER] = ui_gps_logger_create(s_root);
    s_screens[MODE_PBOX] = ui_pbox_create(s_root);
    s_screens[MODE_GNSS_INFO] = ui_gnss_info_create(s_root);
    s_screens[MODE_SETTINGS] = ui_settings_create(s_root);
}

static void refresh_ui(const sensors_state_t *state) {
    ui_telemetry_t telemetry = {
        .sensors = *state,
        .logger_state = gpx_logger_get_state(),
    };
    ui_state_bar_update(s_status_bar, &telemetry);
    for (int i = 0; i < MODE_COUNT; ++i) {
        lv_obj_add_flag(s_screens[i], LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_clear_flag(s_screens[s_ctx.mode], LV_OBJ_FLAG_HIDDEN);
    switch (s_ctx.mode) {
        case MODE_BIKE:
            ui_bike_computer_update(s_screens[MODE_BIKE], &telemetry, s_ctx.ride_distance_km, s_ctx.ride_time_s);
            break;
        case MODE_GPS_LOGGER:
            ui_gps_logger_update(s_screens[MODE_GPS_LOGGER], &telemetry, s_ctx.track_distance_km, s_ctx.track_time_s);
            break;
        case MODE_PBOX:
            ui_pbox_update(s_screens[MODE_PBOX], &telemetry, s_ctx.pbox_status, s_ctx.pbox_target_kmh, s_ctx.pbox_elapsed_s);
            break;
        case MODE_GNSS_INFO:
            ui_gnss_info_update(s_screens[MODE_GNSS_INFO], &telemetry);
            break;
        case MODE_SETTINGS:
            ui_settings_update(s_screens[MODE_SETTINGS], s_ctx.settings_selected, s_ctx.gnss_rate_hz);
            break;
        default:
            break;
    }
}

static void ui_task(void *arg) {
    (void)arg;
    init_lvgl_scene();
    while (1) {
        sensors_state_t state;
        sensors_get_state(&state);
        refresh_ui(&state);
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void) {
    sensors_init();
    diagnostics_init();
    input_manager_init();
    gpx_logger_init();

    xTaskCreate(sensor_task, "sensor", CONFIG_TASK_STACK_DEFAULT, NULL, CONFIG_TASK_PRIO_SENSOR, NULL);
    xTaskCreate(diagnostic_task, "diag", CONFIG_TASK_STACK_DEFAULT, NULL, CONFIG_TASK_PRIO_DIAG, NULL);
    xTaskCreate(input_task, "input_ctl", CONFIG_TASK_STACK_DEFAULT, NULL, CONFIG_TASK_PRIO_INPUT, NULL);
    xTaskCreate(ui_task, "ui", CONFIG_TASK_STACK_DEFAULT * 2, NULL, CONFIG_TASK_PRIO_UI, NULL);

    ESP_LOGI(TAG, "System initialized");
}

