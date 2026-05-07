#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "status_ui.h"

esp_err_t rudder_pot_init(status_write_cb_t status_write, void *status_write_ctx);

uint16_t rudder_pot_get_last_pct(void);

int rudder_pot_get_last_raw(void);
