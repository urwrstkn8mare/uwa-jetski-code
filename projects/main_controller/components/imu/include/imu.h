#pragma once

#include "esp_err.h"
#include <stddef.h>

/**
 * @brief Initialize the ICM-20948 and its DMP.
 *
 * Configures the InvenSense DMP (9-axis orientation / Quat9), enables the FIFO,
 * and prepares the IMU for polling orientation data.
 *
 * @return ESP_OK on success, ESP_FAIL on error.
 */
esp_err_t imu_init(void);

/**
 * @brief Get pitch/roll/yaw from the DMP orientation quaternion.
 *
 * @param[out] pitch Degrees of pitch.
 * @param[out] roll  Degrees of roll.
 * @param[out] yaw   Degrees of yaw.
 * @return ESP_OK on success, ESP_FAIL if not initialized.
 */
esp_err_t imu_get_pitch_roll_yaw(float *pitch, float *roll, float *yaw);

size_t imu_status_line_write(char *buf, size_t cap);
