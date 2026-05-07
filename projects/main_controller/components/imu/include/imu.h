#pragma once

#include "esp_err.h"
#include "status_ui.h"
#include <stddef.h>

esp_err_t imu_init(status_write_cb_t status_write, void *status_write_ctx);

esp_err_t imu_get_pitch_roll_yaw(float *pitch, float *roll, float *yaw);
