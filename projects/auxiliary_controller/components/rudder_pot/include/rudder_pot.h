#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "status_ui.h"

esp_err_t rudder_pot_init(void);

uint16_t rudder_pot_get_last_pct(void);

int rudder_pot_get_last_raw(void);
