#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "esp_err.h"

typedef struct {
    float elevon_left_deg;   /* commanded angle for left elevon */
    float elevon_right_deg;  /* commanded angle for right elevon */
    bool  armed;
    float dbg[6];            /* height_p, i, d — pitch_p, i, d */
} control_output_t;

esp_err_t control_init(const control_config_t *cfg);

void control_set_cfg(const control_config_t *cfg);
void control_get_cfg(control_config_t *cfg);

void control_arm(void);
void control_disarm(void);
bool control_is_armed(void);

void control_register_change_cb(void (*cb)(void));

void control_set_target(int16_t height_cm);
int16_t control_get_target(void);
bool control_get_height_enabled(void);

void control_get_last_output(control_output_t *out);

/* Set the effective servo angle range (derived from servo calibration min/max).
 * Used to compute max elevon_center = max_angle - elevon_max_diff_deg. */
void  control_set_elevon_max_angle(float deg);
float control_get_elevon_max_angle(void);

/* joy_pitch_pct: 0..100, 50 = centre. Used when height_enabled is false to
 * set pitch target directly from the joystick up/down axis. */
void control_update(int16_t height_cm,
                    float pitch_deg,
                    float roll_deg,
                    uint16_t rudder_pct,
                    uint16_t joy_pitch_pct,
                    control_output_t *out);
