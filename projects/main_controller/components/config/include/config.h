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
} control_config_t;

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

esp_err_t config_init(void);

esp_err_t config_load(control_config_t *out);
esp_err_t config_save(const control_config_t *cfg);

void config_get_defaults(control_config_t *out);
