#pragma once

#include "esp_err.h"
#include "lvgl.h"

/**
 * M5 Tab5: enable CAN transceiver 5V, start LVGL display, rotate, backlight, power down unused blocks.
 */
esp_err_t tab5_bringup_init(lv_display_t **out_display);
