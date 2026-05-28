#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef uint8_t servo_channel_t;
#define SERVO_CHANNEL_INVALID UINT8_MAX

#define SERVO_MAX_INSTANCES 8

/* Per-channel servo calibration: pulse-width anchors and the angles they map to.
 * The angle range must span zero (one negative, one positive). */
typedef struct {
    float min_pw_us;
    float zero_pw_us;
    float max_pw_us;
    float min_angle_deg;
    float max_angle_deg;
} servo_calibration_t;

/* Calibration for the two elevon channels (persisted as a unit). */
typedef struct {
    servo_calibration_t channel[2];
} servo_config_t;

#define SERVO_DEFAULT_MIN_PW_US      1300.0f
#define SERVO_DEFAULT_ZERO_PW_US     1500.0f
#define SERVO_DEFAULT_MAX_PW_US      1800.0f
#define SERVO_DEFAULT_MIN_ANGLE_DEG  (-30.0f)
#define SERVO_DEFAULT_MAX_ANGLE_DEG  30.0f

typedef struct {
    bool in_use;
    int gpio;
    bool ready;
    bool simulated;
    bool cal_mode;
    float cmd_deg;
    servo_calibration_t cal;
} servo_info_t;

esp_err_t servo_drive_init_hw(void);

servo_channel_t servo_drive_open(int gpio_num);

/* Set the commanded angle (clamped to the channel's calibrated range) and push
 * it to PWM hardware. Silently ignored if the handle is invalid. */
void servo_drive_set_degrees(servo_channel_t h, float deg);

/* Drive a raw pulse width in microseconds, bypassing the angle calibration.
 * Only honoured while the channel is in calibration mode; otherwise no-op. */
void servo_drive_set_raw_us(servo_channel_t h, float pulse_us);

void servo_drive_set_cal_mode(servo_channel_t h, bool on);

void servo_drive_apply_cal(servo_channel_t h, const servo_calibration_t *cal);

void servo_drive_get_cal(servo_channel_t h, servo_calibration_t *out_cal);

/* Effective usable angle magnitude: the tightest of min(|min_angle|,|max_angle|)
 * across in-use channels. Used by the control loop to bound elevon travel. */
float servo_drive_get_effective_range_deg(void);

bool servo_drive_any_cal_mode(void);

int servo_drive_get_count(void);

bool servo_drive_get_info_by_index(int idx, servo_info_t *out_info);

void servo_drive_register_change_cb(void (*cb)(int idx));