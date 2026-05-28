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
    /* When false, height PID is bypassed and the joystick y axis sets the
     * pitch target directly. */
    bool    height_enabled;
    int16_t elevon_max_diff_deg;  /* max elevon differential (roll authority) */
    int16_t pitch_target_max_deg; /* max pitch setpoint magnitude (height-loop + joystick authority) */
    int16_t height_target_cm;     /* desired hover height in cm, 0..50 */
} control_config_t;

/* Defaults derived from the BeagleBone pilou config (current_feb26.cfg),
 * with pilou's radian/normalised PID output mapped to our degree/elevon units
 * over the default ±30° servo range (kP_uwa ≈ kP_pilou · 30 · π/180 ≈ kP_pilou · 0.524).
 * Height gains are zero to match the config (height loop disabled in pilou). */
#define CONTROL_DEFAULT_HEIGHT_KP  0
#define CONTROL_DEFAULT_HEIGHT_KI  0
#define CONTROL_DEFAULT_HEIGHT_KD  0
#define CONTROL_DEFAULT_PITCH_KP   52
#define CONTROL_DEFAULT_PITCH_KI   0
#define CONTROL_DEFAULT_PITCH_KD   52
#define CONTROL_DEFAULT_ROLL_KP    262
#define CONTROL_DEFAULT_ROLL_KI    0
#define CONTROL_DEFAULT_ROLL_KD    209
#define CONTROL_DEFAULT_RUDDER_EXPONENT_X100  300
#define CONTROL_DEFAULT_RUDDER_MAX_ROLL_DEG   20
#define CONTROL_DEFAULT_HEIGHT_ENABLED        true
#define CONTROL_DEFAULT_ELEVON_MAX_DIFF_DEG   20
#define CONTROL_DEFAULT_PITCH_TARGET_MAX_DEG  10
#define CONTROL_DEFAULT_HEIGHT_TARGET_CM      35

/* Snapshot of control state for display/telemetry consumers. */
typedef struct {
    bool    armed;
    float   elevon_left_deg;
    float   elevon_right_deg;
} control_status_t;

/* Initialise the control loop and start its tasks. Loads persisted config
 * (falling back to defaults), registers a joystick CAN receiver, then starts
 * the high-priority control task (sense → PID → command servos) and a
 * low-priority telemetry task. */
esp_err_t control_init(servo_channel_t servo_left, servo_channel_t servo_right);

/* Apply a new control config: update the running loop and persist it. */
void control_apply_cfg(const control_config_t *cfg);
void control_get_cfg(control_config_t *cfg);
void control_get_defaults(control_config_t *cfg);

void control_arm(void);
void control_disarm(void);

void control_get_status(control_status_t *out);

void control_register_change_cb(void (*cb)(void));
