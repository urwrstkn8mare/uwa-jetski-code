#pragma once

#include "esp_err.h"
#include <stdint.h>

esp_err_t height_init(void);

esp_err_t height_get_cm(int32_t *height_cm);
