#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Persisted IMU configuration. Owned and persisted by the IMU component. */
typedef struct {
    /* Trim offsets added inside imu_get_pitch_roll_yaw so a non-level mount
     * can be zeroed out. Stored in tenths of a degree (e.g. 15 = 1.5°). */
    int16_t pitch_offset_deg_x10;
    int16_t roll_offset_deg_x10;
} imu_config_t;

#define IMU_DEFAULT_PITCH_OFFSET_DEG_X10 0
#define IMU_DEFAULT_ROLL_OFFSET_DEG_X10  0

esp_err_t imu_init(void);
bool imu_is_ready(void);

esp_err_t imu_get_pitch_roll_yaw(float *pitch, float *roll, float *yaw);

/* Live (offset-applied) attitude getter for telemetry consumers — same values
 * imu_get_pitch_roll_yaw returns, but never errors and yields 0 when the IMU
 * is not yet ready. */
void imu_get_attitude(float *pitch, float *roll, float *yaw);

void imu_get_cfg(imu_config_t *cfg);
void imu_get_defaults(imu_config_t *cfg);
/* Apply a new IMU config: update offsets in-place and persist to NVS. */
void imu_apply_cfg(const imu_config_t *cfg);
