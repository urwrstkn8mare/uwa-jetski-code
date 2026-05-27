#include "control.h"

#include "esp_log.h"
#include "status_ui.h"

#include <math.h>
#include <string.h>

static const char *TAG = "control";

static control_config_t s_cfg;
static float s_elevon_max_angle = (-SERVO_DEFAULT_MIN_ANGLE_DEG < SERVO_DEFAULT_MAX_ANGLE_DEG)
                                  ? -SERVO_DEFAULT_MIN_ANGLE_DEG
                                  : SERVO_DEFAULT_MAX_ANGLE_DEG;
static bool s_armed;
static int16_t s_target_height_cm;

/* PID integrator state */
static float s_height_integral;
static float s_height_prev_error;
static bool  s_height_first;

static float s_pitch_integral;
static float s_pitch_prev_error;
static bool  s_pitch_first;

static float s_roll_integral;
static float s_roll_prev_error;
static bool  s_roll_first;

/* Manual pitch setpoint accumulator (rate-mode joystick, height disabled) */
static float s_manual_pitch_target;

static control_output_t s_last_out;
static void (*s_change_cb)(void);

static float apply_pid(float error, float kp, float ki, float kd,
                       float dt, float *integral, float *prev_error, bool *first) {
    float p_term = kp * error;

    float i_term = 0;
    if (ki != 0 && !(*first)) {
        *integral += error * dt;
        i_term = ki * (*integral);
    }

    float d_term = 0;
    if (kd != 0 && !(*first)) {
        float dedt = (error - *prev_error) / dt;
        d_term = kd * dedt;
    }

    if (*first) {
        *first = false;
    }

    *prev_error = error;

    return p_term + i_term + d_term;
}

static void reset_integrals(void) {
    s_height_integral = 0;
    s_height_prev_error = 0;
    s_height_first = true;

    s_pitch_integral = 0;
    s_pitch_prev_error = 0;
    s_pitch_first = true;

    s_roll_integral = 0;
    s_roll_prev_error = 0;
    s_roll_first = true;

    s_manual_pitch_target = 0.0f;
}

esp_err_t control_init(const control_config_t *cfg) {
    if (cfg != NULL) {
        s_cfg = *cfg;
    } else {
        config_get_defaults(&s_cfg);
    }
    s_armed = false;
    s_target_height_cm = 0;
    reset_integrals();
    ESP_LOGI(TAG, "Control initialized, armed=%d target=%d", s_armed, s_target_height_cm);
    return ESP_OK;
}

void control_set_cfg(const control_config_t *cfg) {
    if (cfg) {
        s_cfg = *cfg;
        reset_integrals();
    }
}

void control_get_cfg(control_config_t *cfg) {
    if (cfg) {
        *cfg = s_cfg;
    }
}

void control_register_change_cb(void (*cb)(void)) {
    s_change_cb = cb;
}

void control_set_elevon_max_angle(float deg) {
    if (deg > 0.0f) {
        s_elevon_max_angle = deg;
        status_ui_update("Limits", "max=%.2f", (double)s_elevon_max_angle);
    }
}

float control_get_elevon_max_angle(void) {
    return s_elevon_max_angle;
}

void control_arm(void) {
    if (!s_armed) {
        s_armed = true;
        reset_integrals();
        ESP_LOGI(TAG, "Armed");
        if (s_change_cb) s_change_cb();
    }
}

void control_disarm(void) {
    if (s_armed) {
        s_armed = false;
        reset_integrals();
        s_target_height_cm = 0;
        ESP_LOGI(TAG, "Disarmed");
        if (s_change_cb) s_change_cb();
    }
}

bool control_is_armed(void) {
    return s_armed;
}

void control_set_target(int16_t height_cm) {
    if (height_cm < 0) height_cm = 0;
    if (height_cm > 50) height_cm = 50;
    s_target_height_cm = height_cm;
}

int16_t control_get_target(void) {
    return s_target_height_cm;
}

bool control_get_height_enabled(void) {
    return s_cfg.height_enabled;
}

void control_get_last_output(control_output_t *out) {
    if (out) {
        *out = s_last_out;
    }
}

void control_update(int16_t height_cm,
                    float pitch_deg,
                    float roll_deg,
                    float rudder_angle,
                    uint16_t joy_pitch_pct,
                    control_output_t *out) {
    const float dt = 0.02f;

    if (out) {
        memset(out, 0, sizeof(*out));
    }

    if (!s_armed) return;

    float kp_h = (float)s_cfg.height_kp / 1000.0f;
    float ki_h = (float)s_cfg.height_ki / 1000.0f;
    float kd_h = (float)s_cfg.height_kd / 1000.0f;
    float kp_p = (float)s_cfg.pitch_kp / 1000.0f;
    float ki_p = (float)s_cfg.pitch_ki / 1000.0f;
    float kd_p = (float)s_cfg.pitch_kd / 1000.0f;
    float kp_r = (float)s_cfg.roll_kp / 1000.0f;
    float ki_r = (float)s_cfg.roll_ki / 1000.0f;
    float kd_r = (float)s_cfg.roll_kd / 1000.0f;

    float max_diff   = (float)s_cfg.elevon_max_diff_deg;
    float max_center = s_elevon_max_angle - max_diff;
    if (max_center < 0.0f) max_center = 0.0f;

    /* Joystick always accumulates a pitch trim offset regardless of height mode. */
    float joy_norm = ((float)joy_pitch_pct / 50.0f) - 1.0f;
    if (joy_norm >  1.0f) joy_norm =  1.0f;
    if (joy_norm < -1.0f) joy_norm = -1.0f;
    if (fabsf(joy_norm) < 0.05f) joy_norm = 0.0f;
    s_manual_pitch_target += joy_norm * 20.0f * dt;
    if (s_manual_pitch_target >  max_center) s_manual_pitch_target =  max_center;
    if (s_manual_pitch_target < -max_center) s_manual_pitch_target = -max_center;

    float pitch_target;
    if (s_cfg.height_enabled) {
        float height_error = (float)(s_target_height_cm - height_cm);
        pitch_target = apply_pid(height_error, kp_h, ki_h, kd_h, dt,
                                 &s_height_integral, &s_height_prev_error, &s_height_first);
        pitch_target += s_manual_pitch_target;
    } else {
        pitch_target = s_manual_pitch_target;
        s_height_integral   = 0.0f;
        s_height_prev_error = 0.0f;
        s_height_first      = true;
    }

    if (pitch_target >  15.0f) pitch_target =  15.0f;
    if (pitch_target < -15.0f) pitch_target = -15.0f;

    float pitch_error   = pitch_target - pitch_deg;
    float elevon_center = apply_pid(pitch_error, kp_p, ki_p, kd_p, dt,
                                    &s_pitch_integral, &s_pitch_prev_error, &s_pitch_first);
    if (elevon_center >  max_center) elevon_center =  max_center;
    if (elevon_center < -max_center) elevon_center = -max_center;

    float rudder_exp = (float)s_cfg.rudder_exponent_x100 / 100.0f;
    float max_roll = (float)s_cfg.rudder_max_roll_deg;
    float rudder_norm = (max_roll > 0.0f) ? (rudder_angle / max_roll) : 0.0f;
    if (rudder_norm >  1.0f) rudder_norm =  1.0f;
    if (rudder_norm < -1.0f) rudder_norm = -1.0f;
    float abs_norm = fabsf(rudder_norm);
    float roll_target_deg = powf(abs_norm, rudder_exp) * max_roll;
    if (rudder_norm < 0.0f) roll_target_deg = -roll_target_deg;

    float roll_error = roll_target_deg - roll_deg;
    float elevon_diff = apply_pid(roll_error, kp_r, ki_r, kd_r, dt,
                                  &s_roll_integral, &s_roll_prev_error, &s_roll_first);

    if (elevon_diff >  max_diff) elevon_diff =  max_diff;
    if (elevon_diff < -max_diff) elevon_diff = -max_diff;

    float elevon_left = elevon_center + elevon_diff;
    float elevon_right = elevon_center - elevon_diff;

    if (out) {
        out->armed = true;
        out->elevon_left_deg  = elevon_left;
        out->elevon_right_deg = elevon_right;
        out->pitch_target_deg = pitch_target;
        out->roll_target_deg  = roll_target_deg;
        s_last_out = *out;
    }

    status_ui_update("Limits", "max=%.2f ctr=%.2f diff=%.2f",
                     (double)s_elevon_max_angle, (double)max_center, (double)max_diff);
    status_ui_update("Control", "%s target=%d ht=%d L=%.2f R=%.2f",
                     "ARMED",
                     (int)s_target_height_cm, (int)height_cm,
                     (double)(out ? out->elevon_left_deg : 0.0f),
                     (double)(out ? out->elevon_right_deg : 0.0f));
    status_ui_update("Elevon", "C=%.2f D=%.2f", (double)elevon_center, (double)elevon_diff);
    status_ui_update("Setpts", "ht=%d pt=%.2f rl=%.2f",
                     (int)s_target_height_cm, (double)pitch_target, (double)roll_target_deg);
}
