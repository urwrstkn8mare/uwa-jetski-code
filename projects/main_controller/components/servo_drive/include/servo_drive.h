#pragma once

#include <stdint.h>

#include "esp_err.h"

/**
 * Dual mirrored 50 Hz LEDC outputs from the same rudder *position* demand (POT % on CAN).
 * (Not rudder-angle sensing — that comes from POT on the auxiliary node.)
 */
esp_err_t servo_drive_init(void);

void servo_drive_set_pct(uint16_t pot_pct_0_100);

/** Degrees implied by the last pulse (same mapping on A and B; for CAN elevon echo). */
void servo_drive_get_commanded_deg(int16_t *deg_a, int16_t *deg_b);
