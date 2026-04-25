#pragma once

#include "esp_err.h"

/**
 * @brief Initialize the IMU and zero the sensor.
 *
 * This function blocks until the DMP is ready and the sensor has been zeroed.
 * Keep the sensor stationary during this call.
 *
 * @return ESP_OK on success, ESP_FAIL on error.
 */
esp_err_t imu_init(void);

/**
 * @brief Get the current pitch and roll relative to the zeroed orientation.
 *
 * @param[out] pitch Degrees of pitch relative to zero.
 * @param[out] roll  Degrees of roll relative to zero.
 * @return ESP_OK on success, ESP_FAIL if not initialized.
 */
esp_err_t imu_get_pitch_roll(float *pitch, float *roll);
