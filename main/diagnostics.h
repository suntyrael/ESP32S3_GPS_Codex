#pragma once

#include "sensors.h"

void diagnostics_init(void);
void diagnostics_report_boot(const sensors_state_t *state, uint32_t uptime_ms);
void diagnostics_report_heartbeat(const sensors_state_t *state, uint32_t uptime_ms);
void diagnostics_trigger_event(const char *event_name, uint32_t duration_ms);

