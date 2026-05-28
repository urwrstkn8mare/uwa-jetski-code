#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "servo_drive.h"

/* PID gains stored as int32_t * 1000. At runtime: effective_gain = value / 1000.0 */
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
    /* When false, height PID is bypassed and the joystick pitch axis sets the
     * pitch target directly. */
    bool    height_enabled;
    int16_t elevon_max_diff_deg;  /* max elevon differential (roll authority) */
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
#define CONTROL_DEFAULT_HEIGHT_ENABLED        true
#define CONTROL_DEFAULT_ELEVON_MAX_DIFF_DEG   5

/* Snapshot of control state for display/telemetry consumers. */
typedef struct {
    bool    armed;
    int16_t target_cm;
    float   elevon_left_deg;
    float   elevon_right_deg;
    bool    height_enabled;
} control_status_t;

/* Initialise the control loop and start its tasks.
 *
 * Loads the persisted control config (falling back to defaults), registers a
 * joystick CAN receiver, then starts the high-priority control task
 * (sense → PID → command servos) and a low-priority telemetry task. The two
 * servo channels are commanded by the control task; the caller retains
 * ownership for calibration. */
esp_err_t control_init(servo_channel_t servo_left, servo_channel_t servo_right);

/* Apply a new control config: update the running loop and persist it. */
void control_apply_cfg(const control_config_t *cfg);
void control_get_cfg(control_config_t *cfg);

void control_arm(void);
void control_disarm(void);

void control_set_target(int16_t height_cm);

void control_get_status(control_status_t *out);

void control_register_change_cb(void (*cb)(void));
