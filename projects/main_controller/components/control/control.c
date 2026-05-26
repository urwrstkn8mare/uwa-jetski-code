#include "control.h"

#include "can.h"
#include "can_ids.h"
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
    if (deg > 0.0f) s_elevon_max_angle = deg;
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

    if (s_target_height_cm >= s_cfg.arm_threshold_pct / 2 && !s_armed) {
        control_arm();
    } else if (s_target_height_cm <= s_cfg.disarm_threshold_pct / 2 && s_armed) {
        control_disarm();
    } else {
        if (s_change_cb) s_change_cb();
    }
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
                    uint16_t rudder_pct,
                    uint16_t joy_pitch_pct,
                    control_output_t *out) {
    const float dt = 0.02f;

    if (out) {
        memset(out, 0, sizeof(*out));
    }

    if (!s_armed) {
        if (out) {
            out->elevon_left_deg = 0.0f;
            out->elevon_right_deg = 0.0f;
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

    float pitch_target;
    if (s_cfg.height_enabled) {
        float height_error = (float)(s_target_height_cm - height_cm);
        pitch_target = apply_pid(height_error, kp_h, ki_h, kd_h, dt,
                                 &s_height_integral, &s_height_prev_error, &s_height_first);
    } else {
        /* Joystick pitch axis: 0..100 → ±(elevon_max_angle - elevon_max_diff_deg) */
        float joy_norm = ((float)joy_pitch_pct / 50.0f) - 1.0f;
        if (joy_norm >  1.0f) joy_norm =  1.0f;
        if (joy_norm < -1.0f) joy_norm = -1.0f;
        float max_center = s_elevon_max_angle - (float)s_cfg.elevon_max_diff_deg;
        if (max_center < 0.0f) max_center = 0.0f;
        pitch_target = joy_norm * max_center;
        /* Keep height integrator zeroed when bypassed */
        s_height_integral  = 0;
        s_height_prev_error = 0;
        s_height_first = true;
    }

    if (pitch_target > 15.0f) pitch_target = 15.0f;
    if (pitch_target < -15.0f) pitch_target = -15.0f;

    float pitch_error = pitch_target - pitch_deg;
    float elevon_center = apply_pid(pitch_error, kp_p, ki_p, kd_p, dt,
                                    &s_pitch_integral, &s_pitch_prev_error, &s_pitch_first);

    float max_diff   = (float)s_cfg.elevon_max_diff_deg;
    float max_center = s_elevon_max_angle - max_diff;
    if (max_center < 0.0f) max_center = 0.0f;

    if (elevon_center >  max_center) elevon_center =  max_center;
    if (elevon_center < -max_center) elevon_center = -max_center;

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

    if (elevon_diff >  max_diff) elevon_diff =  max_diff;
    if (elevon_diff < -max_diff) elevon_diff = -max_diff;

    float elevon_left = elevon_center + elevon_diff;
    float elevon_right = elevon_center - elevon_diff;

    if (out) {
        out->armed = true;
        out->elevon_left_deg = elevon_left;
        out->elevon_right_deg = elevon_right;
        s_last_out = *out;
    }

    can_ctrl_status_t cs = {
        .height_target_cm      = (uint8_t)(s_target_height_cm < 0   ? 0   :
                                            s_target_height_cm > 100 ? 100 :
                                            s_target_height_cm),
        .height_current_cm_x10 = (uint16_t)(height_cm * 10),
        .pitch_target_deg_x10  = (int16_t)(pitch_target * 10.0f),
        .roll_target_deg_x10   = (int16_t)(mapped_roll  * 10.0f),
        .flags                 = 1u,
    };
    (void)can_tx(CAN_ID_CTRL_STATUS, (const uint8_t *)&cs, sizeof(cs));

    status_ui_update("Control", "%s target=%d ht=%d L=%.1f R=%.1f",
                     "ARMED",
                     (int)s_target_height_cm, (int)height_cm,
                     (double)(out ? out->elevon_left_deg : 0.0f),
                     (double)(out ? out->elevon_right_deg : 0.0f));
}
