#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/* PID gains stored as int32_t * 1000.
 * At runtime: effective_gain = value / 1000.0 */

typedef struct {
    int32_t height_kp;
    int32_t height_ki;
    int32_t height_kd;
    int32_t pitch_kp;
    int32_t pitch_ki;
    int32_t pitch_kd;
    int32_t roll_kp;
    int32_t roll_ki;
    int32_t roll_kd;
    int16_t rudder_exponent_x100;
    int16_t rudder_max_roll_deg;
    int16_t arm_threshold_pct;
    int16_t disarm_threshold_pct;
    /* When false, height PID is bypassed and the joystick pitch axis sets the
     * pitch target directly (range ±(elevon_max_angle - elevon_max_diff_deg)). */
    bool    height_enabled;
    int16_t elevon_max_diff_deg;  /* max elevon differential (roll authority) */
} control_config_t;

typedef struct {
    float min_pw_us;
    float zero_pw_us;
    float max_pw_us;
    float min_angle_deg;
    float max_angle_deg;
} servo_calibration_t;

typedef struct {
    servo_calibration_t channel[2];
} servo_config_t;

typedef struct {
    control_config_t control;
    servo_config_t servo;
} app_config_t;

#define CONTROL_DEFAULT_HEIGHT_KP  100
#define CONTROL_DEFAULT_HEIGHT_KI  0
#define CONTROL_DEFAULT_HEIGHT_KD  20
#define CONTROL_DEFAULT_PITCH_KP   1000
#define CONTROL_DEFAULT_PITCH_KI   0
#define CONTROL_DEFAULT_PITCH_KD   50
#define CONTROL_DEFAULT_ROLL_KP    300
#define CONTROL_DEFAULT_ROLL_KI    0
#define CONTROL_DEFAULT_ROLL_KD    10

#define CONTROL_DEFAULT_RUDDER_EXPONENT_X100  200
#define CONTROL_DEFAULT_RUDDER_MAX_ROLL_DEG   20
#define CONTROL_DEFAULT_ARM_THRESHOLD_PCT     50
#define CONTROL_DEFAULT_DISARM_THRESHOLD_PCT  30
#define CONTROL_DEFAULT_HEIGHT_ENABLED        true
#define CONTROL_DEFAULT_ELEVON_MAX_DIFF_DEG   5

#define SERVO_DEFAULT_MIN_PW_US      1300.0f
#define SERVO_DEFAULT_ZERO_PW_US     1500.0f
#define SERVO_DEFAULT_MAX_PW_US      1800.0f
#define SERVO_DEFAULT_MIN_ANGLE_DEG  (-8.0f)
#define SERVO_DEFAULT_MAX_ANGLE_DEG  12.0f

esp_err_t config_init(void);

esp_err_t config_load(app_config_t *out);
esp_err_t config_save(const app_config_t *cfg);

esp_err_t config_save_servo_cal(int channel_idx, const servo_calibration_t *cal);
esp_err_t config_save_control_cfg(const control_config_t *cfg);

void config_get_defaults(control_config_t *out);
void config_get_servo_defaults(servo_config_t *out);
void config_get_app_defaults(app_config_t *out);
