#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

esp_err_t height_init(void);
bool height_is_ready(void);
bool height_is_simulated(void);

esp_err_t height_get_cm(int32_t *height_cm);
