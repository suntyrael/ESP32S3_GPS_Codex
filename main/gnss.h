#pragma once

#include "sensors.h"

void gnss_init(void);
void gnss_poll(gnss_state_t *state);
