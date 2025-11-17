#pragma once

#include "sensors.h"

void gnss_init(void);
void gnss_poll(gnss_state_t *state);
bool gnss_set_update_rate(uint8_t hz);
bool gnss_set_constellations(uint8_t mask);
