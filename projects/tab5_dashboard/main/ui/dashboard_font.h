#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "lvgl.h"

esp_err_t dashboard_font_get(uint16_t size_px, int weight, const lv_font_t **font);
