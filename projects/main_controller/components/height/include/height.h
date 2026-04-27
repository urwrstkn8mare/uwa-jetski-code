#pragma once

#include "esp_err.h"
#include <stdint.h>

/**
 * @brief Initialize the A02YYUW ultrasonic height sensor.
 *
 * This function blocks until the UART reader task is ready.
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
