#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/** Rudder demand (POT %) drives direct PWM servos on GPIO1/GPIO2. */
esp_err_t servo_drive_init(void);

/** False if PWM init failed — set_pct becomes a safe cache-only no-op. */
bool servo_drive_is_ready(void);

void servo_drive_set_pct(uint16_t pot_pct_0_100);

/** Degrees implied by the midpoint of the last two commanded pulses — CAN elevon echo. */
void servo_drive_get_commanded_deg(int16_t *deg_a, int16_t *deg_b);

void servo_drive_get_pulse_us(uint32_t *ch0_us_out, uint32_t *ch1_us_out);
bool servo_drive_get_feedback_us(uint8_t channel, uint16_t *pulse_us_out);
