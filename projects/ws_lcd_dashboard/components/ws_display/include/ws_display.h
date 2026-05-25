#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct _lv_font_t lv_font_t;

#define WS_DISPLAY_H_RES 1024
#define WS_DISPLAY_V_RES 600

/**
 * @brief Supported font weights.
 */
typedef enum {
  WS_DISPLAY_FONT_WEIGHT_REGULAR = 0,
  WS_DISPLAY_FONT_WEIGHT_SEMIBOLD,
} ws_display_font_weight_t;

/**
 * @brief Initialize the display subsystem.
 *
 * Sets up the RGB panel, LVGL adapter, and font assets.
 */
esp_err_t ws_display_init(void);

/**
 * @brief Acquire the display lock.
 *
 * Hold this lock before touching LVGL objects or creating fonts.
 *
 * @param timeout_ms Lock timeout in milliseconds; pass portMAX_DELAY to wait forever.
 */
bool ws_display_lock(uint32_t timeout_ms);

/**
 * @brief Release the display lock.
 */
void ws_display_unlock(void);

/**
 * @brief Create a font variant and return its LVGL font.
 *
 * The caller must hold `ws_display_lock()` while calling this function.
 * Store the returned font pointer if you need to reuse the variant later.
 * This function does not cache fonts.
 *
 * @param size_px Font size in pixels.
 * @param weight Font weight.
 * @param[out] font Receives the LVGL font pointer.
 */
esp_err_t ws_display_font_get(uint16_t size_px, ws_display_font_weight_t weight,
                              const lv_font_t **font);
