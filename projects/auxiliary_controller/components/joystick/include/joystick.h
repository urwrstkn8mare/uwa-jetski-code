#pragma once

#include "esp_err.h"
#include <stdint.h>

/* Initialise the joystick ADC and start the CAN TX task.
 * Reads two potentiometers (bank and pitch axes) and broadcasts
 * CAN_ID_JOYSTICK at 20 Hz when the main controller is not armed.
 * Returns ESP_FAIL if CONFIG_JOYSTICK_SKIP_HW is set. */
esp_err_t joystick_init(void);

/* Called from the application's CAN RX callback to detect armed state.
 * Suppresses joystick TX when the main controller reports armed. */
void joystick_on_can_rx(const uint8_t buffer[8], uint32_t header_id);

uint16_t joystick_get_bank_pct(void);
uint16_t joystick_get_pitch_pct(void);
int      joystick_get_bank_raw(void);
int      joystick_get_pitch_raw(void);
