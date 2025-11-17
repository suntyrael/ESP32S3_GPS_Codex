#pragma once

#include "ui_common.h"

typedef enum {
    PBOX_STATUS_READY,
    PBOX_STATUS_ARMED,
    PBOX_STATUS_RUNNING,
    PBOX_STATUS_FINISHED
} pbox_status_t;

lv_obj_t *ui_pbox_create(lv_obj_t *parent);
void ui_pbox_update(lv_obj_t *screen, const ui_telemetry_t *telemetry,
                    pbox_status_t status, float current_target_kmh, float elapsed_s);

