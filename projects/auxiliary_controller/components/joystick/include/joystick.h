#pragma once

#include "esp_err.h"
#include <stdint.h>

/* Initialise the joystick ADC and start the CAN TX task.
 * Reads two potentiometers (bank and pitch axes) and broadcasts
 * CAN_ID_JOYSTICK at 20 Hz continuously.
 * Returns ESP_FAIL if CONFIG_JOYSTICK_SKIP_HW is set. */
esp_err_t joystick_init(void);

uint16_t joystick_get_bank_pct(void);
uint16_t joystick_get_pitch_pct(void);
int      joystick_get_bank_raw(void);
int      joystick_get_pitch_raw(void);
