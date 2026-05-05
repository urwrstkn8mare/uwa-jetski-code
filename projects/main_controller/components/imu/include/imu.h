#pragma once

#include "esp_err.h"

/**
 * @brief Initialize the ICM-20948 and its DMP, then zero the sensor.
 *
 * Configures the InvenSense DMP (9-axis orientation / Quat9), enables the FIFO,
 * and blocks until the DMP is running and a zero reference is captured. Keep
 * the board still during this call.
 *
 * @return ESP_OK on success, ESP_FAIL on error.
 */
esp_err_t imu_init(void);

/**
 * @brief Get pitch and roll from the DMP orientation quaternion (relative to zero).
 *
 * @param[out] pitch Degrees of pitch relative to zero.
 * @param[out] roll  Degrees of roll relative to zero.
 * @return ESP_OK on success, ESP_FAIL if not initialized.
 */
esp_err_t imu_get_pitch_roll(float *pitch, float *roll);

/**
 * @brief Get pitch/roll/yaw from the DMP orientation quaternion (relative to zero).
 *
 * @param[out] pitch Degrees of pitch relative to zero.
 * @param[out] roll  Degrees of roll relative to zero.
 * @param[out] yaw   Degrees of yaw relative to zero.
 * @return ESP_OK on success, ESP_FAIL if not initialized.
 */
esp_err_t imu_get_pitch_roll_yaw(float *pitch, float *roll, float *yaw);
