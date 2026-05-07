#pragma once

#include "esp_err.h"
#include <stddef.h>

esp_err_t imu_init(void);

esp_err_t imu_get_pitch_roll_yaw(float *pitch, float *roll, float *yaw);
