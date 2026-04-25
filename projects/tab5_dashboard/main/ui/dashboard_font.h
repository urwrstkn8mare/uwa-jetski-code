#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "lvgl.h"

typedef enum {
    DASHBOARD_FONT_WEIGHT_REGULAR = 0,
    DASHBOARD_FONT_WEIGHT_SEMIBOLD = 1,
} dashboard_font_weight_t;

esp_err_t dashboard_font_get(uint16_t size_px, dashboard_font_weight_t weight, const lv_font_t **font);
