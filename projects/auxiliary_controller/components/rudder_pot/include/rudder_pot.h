#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/**
 * Samples rudder pot on ADC and transmits CAN_ID_POTENTIOMETER at ~20 Hz.
 */
esp_err_t rudder_pot_init(void);

/** Last scaled demand 0..100 (same value sent on CAN), for local status UI. */
uint16_t rudder_pot_get_last_pct(void);

/** Last raw ADC sample, for calibration/debug UI. */
int rudder_pot_get_last_raw(void);

size_t rudder_pot_status_line_write(char *buf, size_t cap);
