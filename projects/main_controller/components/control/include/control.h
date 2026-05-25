#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "esp_err.h"

typedef struct {
    float   elevon_left_deg;       /* commanded angle for left elevon */
    float   elevon_right_deg;      /* commanded angle for right elevon */
    int16_t height_pitch_target;   /* pitch target from height PID (deg*10) */
    bool    armed;
    float   dbg[6];                /* height_p, i, d — pitch_p, i, d */
} control_output_t;

esp_err_t control_init(const control_config_t *cfg);

void control_set_cfg(const control_config_t *cfg);
void control_get_cfg(control_config_t *cfg);

void control_arm(void);
void control_disarm(void);
bool control_is_armed(void);

void control_register_arm_cb(void (*cb)(void));
void control_register_change_cb(void (*cb)(void));

void control_set_target(int16_t height_cm);
int16_t control_get_target(void);

void control_get_last_output(control_output_t *out);

void control_update(int16_t height_cm,
                    float pitch_deg,
                    float roll_deg,
                    uint16_t rudder_pct,
                    control_output_t *out);
