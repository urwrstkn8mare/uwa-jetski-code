#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "status_ui.h"

esp_err_t servo_drive_init(void);

bool servo_drive_is_ready(void);
bool servo_drive_is_simulated(void);

void servo_drive_set_channels(uint32_t ch0_us, uint32_t ch1_us);

void servo_drive_get_commanded_deg(int16_t *deg_a, int16_t *deg_b);

void servo_drive_get_pulse_us(uint32_t *ch0_us_out, uint32_t *ch1_us_out);
