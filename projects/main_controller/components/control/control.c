#include "control.h"

#include "esp_log.h"

#include <math.h>
#include <string.h>

static const char *TAG = "control";

static control_config_t s_cfg;
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

static control_output_t s_last_out;

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

void control_arm(void) {
    if (!s_armed) {
        s_armed = true;
        reset_integrals();
        ESP_LOGI(TAG, "Armed");
    }
}

void control_disarm(void) {
    if (s_armed) {
        s_armed = false;
        reset_integrals();
        s_target_height_cm = 0;
        ESP_LOGI(TAG, "Disarmed");
    }
}

bool control_is_armed(void) {
    return s_armed;
}

void control_set_target(int16_t height_cm) {
    if (height_cm < 0) height_cm = 0;
    if (height_cm > 50) height_cm = 50;
    s_target_height_cm = height_cm;

    if (s_target_height_cm >= s_cfg.arm_threshold_pct / 2 && !s_armed) {
        control_arm();
    } else if (s_target_height_cm <= s_cfg.disarm_threshold_pct / 2 && s_armed) {
        control_disarm();
    }
}

int16_t control_get_target(void) {
    return s_target_height_cm;
}

void control_get_last_output(control_output_t *out) {
    if (out) {
        *out = s_last_out;
    }
}

void control_update(int16_t height_cm,
                    float pitch_deg,
                    float roll_deg,
                    uint16_t rudder_pct,
                    control_output_t *out) {
    const float dt = 0.02f;

    if (out) {
        memset(out, 0, sizeof(*out));
    }

    if (!s_armed) {
        if (out) {
            out->elevon_left_deg = 0;
            out->elevon_right_deg = 0;
            out->armed = false;
        }
        return;
    }

    float kp_h = (float)s_cfg.height_kp / 1000.0f;
    float ki_h = (float)s_cfg.height_ki / 1000.0f;
    float kd_h = (float)s_cfg.height_kd / 1000.0f;
    float kp_p = (float)s_cfg.pitch_kp / 1000.0f;
    float ki_p = (float)s_cfg.pitch_ki / 1000.0f;
    float kd_p = (float)s_cfg.pitch_kd / 1000.0f;
    float kp_r = (float)s_cfg.roll_kp / 1000.0f;
    float ki_r = (float)s_cfg.roll_ki / 1000.0f;
    float kd_r = (float)s_cfg.roll_kd / 1000.0f;

    float height_error = (float)(s_target_height_cm - height_cm);
    float pitch_target = apply_pid(height_error, kp_h, ki_h, kd_h, dt,
                                   &s_height_integral, &s_height_prev_error, &s_height_first);

    if (pitch_target > 15.0f) pitch_target = 15.0f;
    if (pitch_target < -15.0f) pitch_target = -15.0f;

    float pitch_error = pitch_target - pitch_deg;
    float elevon_center = apply_pid(pitch_error, kp_p, ki_p, kd_p, dt,
                                    &s_pitch_integral, &s_pitch_prev_error, &s_pitch_first);

    if (elevon_center > 50.0f) elevon_center = 50.0f;
    if (elevon_center < -50.0f) elevon_center = -50.0f;

    float rudder_exp = (float)s_cfg.rudder_exponent_x100 / 100.0f;
    float rudder_norm = ((float)rudder_pct / 50.0f) - 1.0f;
    if (rudder_norm > 1.0f) rudder_norm = 1.0f;
    if (rudder_norm < -1.0f) rudder_norm = -1.0f;
    float abs_norm = fabsf(rudder_norm);
    float mapped_roll = powf(abs_norm, rudder_exp) * (float)s_cfg.rudder_max_roll_deg;
    if (rudder_norm < 0) mapped_roll = -mapped_roll;

    float roll_error = mapped_roll - roll_deg;
    float elevon_diff = apply_pid(roll_error, kp_r, ki_r, kd_r, dt,
                                  &s_roll_integral, &s_roll_prev_error, &s_roll_first);

    if (elevon_diff > 25.0f) elevon_diff = 25.0f;
    if (elevon_diff < -25.0f) elevon_diff = -25.0f;

    float elevon_left = elevon_center + elevon_diff;
    float elevon_right = elevon_center - elevon_diff;

    if (out) {
        out->armed = true;
        out->elevon_left_deg = (int16_t)(elevon_left);
        out->elevon_right_deg = (int16_t)(elevon_right);
        out->height_pitch_target = (int16_t)(pitch_target * 10.0f);
        s_last_out = *out;
    }
}
