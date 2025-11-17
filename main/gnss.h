#pragma once

#include "sensors.h"
#include "gnss_types.h"

void gnss_init(void);
void gnss_poll(gnss_state_t *state);
bool gnss_set_update_rate(uint8_t hz);
bool gnss_set_constellations(uint8_t mask);
bool gnss_set_dynamic_mode(gnss_dynamic_mode_t mode);
const char *gnss_dynamic_mode_label(gnss_dynamic_mode_t mode);
