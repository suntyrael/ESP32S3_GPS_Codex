#include "gpx_logger.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "config.h"

typedef enum {
    GPX_EVENT_SAMPLE,
    GPX_EVENT_START,
    GPX_EVENT_STOP,
} gpx_event_type_t;

typedef struct {
    gpx_event_type_t type;
    sensors_state_t snapshot;
} gpx_event_t;

static const char *TAG = "gpx";
static gpx_logger_state_t s_state = GPX_LOGGER_STATE_IDLE;
static gpx_logger_state_t s_requested_state = GPX_LOGGER_STATE_IDLE;
static QueueHandle_t s_sample_queue;
static FILE *s_file;
static uint32_t s_file_counter = 1;

static void ensure_directory(void) {
    struct stat st;
    if (stat(CONFIG_GPX_DIRECTORY, &st) != 0) {
        if (mkdir(CONFIG_GPX_DIRECTORY, 0775) != 0) {
            ESP_LOGW(TAG, "mkdir %s failed (%d)", CONFIG_GPX_DIRECTORY, errno);
        }
    }
}

static bool open_new_file(void) {
    ensure_directory();
    char path[64];
    for (uint32_t idx = s_file_counter; idx < 10000; ++idx) {
        snprintf(path, sizeof(path), "%s/%s%04u.gpx", CONFIG_GPX_DIRECTORY, CONFIG_GPX_FILE_PREFIX, idx);
        struct stat st;
        if (stat(path, &st) == 0) {
            continue;
        }
        s_file = fopen(path, "w");
        if (s_file) {
            s_file_counter = idx + 1;
            ESP_LOGI(TAG, "Recording GPX to %s", path);
            fprintf(s_file,
                    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                    "<gpx version=\"1.1\" creator=\"ESP32-S3\" xmlns=\"http://www.topografix.com/GPX/1/1\" xmlns:%s=\"https://espressif.com/telemetry\">\n"
                    "<trk><name>Activity</name><trkseg>\n",
                    CONFIG_GPX_NAMESPACE);
            fflush(s_file);
            return true;
        }
        ESP_LOGE(TAG, "Failed to open %s (%d)", path, errno);
        break;
    }
    return false;
}

static void close_file(void) {
    if (!s_file) {
        return;
    }
    fprintf(s_file, "</trkseg></trk></gpx>\n");
    fflush(s_file);
    fclose(s_file);
    s_file = NULL;
}

static void write_sample(const sensors_state_t *state) {
    if (!s_file || s_state != GPX_LOGGER_STATE_RECORDING) {
        return;
    }
    float g_total = sqrtf(state->imu.linear_accel_g.x * state->imu.linear_accel_g.x +
                          state->imu.linear_accel_g.y * state->imu.linear_accel_g.y +
                          state->imu.linear_accel_g.z * state->imu.linear_accel_g.z);
    fprintf(s_file,
            "<trkpt lat=\"%.6f\" lon=\"%.6f\"><ele>%.1f</ele>\n"
            "<extensions><%s:temperature>%.2f</%s:temperature><%s:g_total>%.3f</%s:g_total>"
            "<%s:gx>%.3f</%s:gx><%s:gy>%.3f</%s:gy><%s:gz>%.3f</%s:gz>"
            "<%s:pressure>%.2f</%s:pressure></extensions></trkpt>\n",
            state->gnss.latitude_deg,
            state->gnss.longitude_deg,
            state->gnss.altitude_m,
            CONFIG_GPX_NAMESPACE, state->imu.temperature.temperature_c, CONFIG_GPX_NAMESPACE,
            CONFIG_GPX_NAMESPACE, g_total, CONFIG_GPX_NAMESPACE,
            CONFIG_GPX_NAMESPACE, state->imu.linear_accel_g.x, CONFIG_GPX_NAMESPACE,
            CONFIG_GPX_NAMESPACE, state->imu.linear_accel_g.y, CONFIG_GPX_NAMESPACE,
            CONFIG_GPX_NAMESPACE, state->imu.linear_accel_g.z, CONFIG_GPX_NAMESPACE,
            CONFIG_GPX_NAMESPACE, state->baro.pressure_hpa, CONFIG_GPX_NAMESPACE);
    fflush(s_file);
}

static void gpx_task(void *arg) {
    (void)arg;
    gpx_event_t event;
    while (1) {
        if (xQueueReceive(s_sample_queue, &event, portMAX_DELAY) == pdTRUE) {
            switch (event.type) {
                case GPX_EVENT_START:
                    if (open_new_file()) {
                        s_state = GPX_LOGGER_STATE_RECORDING;
                        s_requested_state = GPX_LOGGER_STATE_RECORDING;
                    } else {
                        s_requested_state = GPX_LOGGER_STATE_IDLE;
                    }
                    break;
                case GPX_EVENT_STOP:
                    close_file();
                    s_state = GPX_LOGGER_STATE_IDLE;
                    s_requested_state = GPX_LOGGER_STATE_IDLE;
                    break;
                case GPX_EVENT_SAMPLE:
                    write_sample(&event.snapshot);
                    break;
                default:
                    break;
            }
        }
    }
}

void gpx_logger_init(void) {
    s_sample_queue = xQueueCreate(CONFIG_GPX_SAMPLE_QUEUE_DEPTH, sizeof(gpx_event_t));
    xTaskCreatePinnedToCore(gpx_task, "gpx", CONFIG_TASK_STACK_DEFAULT, NULL, CONFIG_TASK_PRIO_LOGGER, NULL, 0);
    ESP_LOGI(TAG, "GPX logger ready at %s", CONFIG_GPX_DIRECTORY);
}

void gpx_logger_start(void) {
    if (!s_sample_queue) {
        return;
    }
    if (s_requested_state == GPX_LOGGER_STATE_RECORDING) {
        return;
    }
    s_requested_state = GPX_LOGGER_STATE_RECORDING;
    gpx_event_t evt = {.type = GPX_EVENT_START};
    xQueueSend(s_sample_queue, &evt, 0);
}

void gpx_logger_stop(void) {
    if (!s_sample_queue) {
        return;
    }
    if (s_requested_state == GPX_LOGGER_STATE_IDLE) {
        return;
    }
    s_requested_state = GPX_LOGGER_STATE_IDLE;
    gpx_event_t evt = {.type = GPX_EVENT_STOP};
    xQueueSend(s_sample_queue, &evt, 0);
}

void gpx_logger_push_sample(const sensors_state_t *state) {
    if (!state || !s_sample_queue) {
        return;
    }
    if (s_requested_state != GPX_LOGGER_STATE_RECORDING) {
        return;
    }
    gpx_event_t event = {
        .type = GPX_EVENT_SAMPLE,
        .snapshot = *state,
    };
    xQueueSend(s_sample_queue, &event, 0);
}

gpx_logger_state_t gpx_logger_get_state(void) {
    return s_requested_state;
}

