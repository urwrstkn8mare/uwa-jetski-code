#pragma once

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Initialize the A02YYUW ultrasonic height sensor (if replies on UART).
 *
 * Blocks briefly while a task probes ~2 s for valid distance frames; unplugged =
 * graceful failure (`ESP_FAIL`), frees UART — UI shows “ultrasonic N/C”.
 *
 * @return ESP_OK on success, ESP_FAIL on error.
 */
esp_err_t height_init(void);

/**
 * @brief Get the current height in centimetres.
 *
 * @param[out] height_cm Height in centimetres.
 * @return ESP_OK on success, ESP_FAIL if not initialized.
 */
esp_err_t height_get_cm(int32_t *height_cm);

/** One LVGL/debug line from the ultrasonic driver (no newline). */
size_t height_status_line_write(char *buf, size_t cap);
