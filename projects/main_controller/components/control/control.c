#include "control.h"

#include "can.h"
#include "can_ids.h"
#include "config.h"
#include "encoder_can.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "height.h"
#include "imu.h"
#include "servo_drive.h"
#include "status_ui.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>

static const char *TAG = "control";

#define CONTROL_NVS_KEY "control"

static const control_config_t s_control_defaults = {
    .height_kp = CONTROL_DEFAULT_HEIGHT_KP,
    .height_ki = CONTROL_DEFAULT_HEIGHT_KI,
    .height_kd = CONTROL_DEFAULT_HEIGHT_KD,
    .pitch_kp  = CONTROL_DEFAULT_PITCH_KP,
    .pitch_ki  = CONTROL_DEFAULT_PITCH_KI,
    .pitch_kd  = CONTROL_DEFAULT_PITCH_KD,
    .roll_kp   = CONTROL_DEFAULT_ROLL_KP,
    .roll_ki   = CONTROL_DEFAULT_ROLL_KI,
    .roll_kd   = CONTROL_DEFAULT_ROLL_KD,
    .rudder_exponent_x100  = CONTROL_DEFAULT_RUDDER_EXPONENT_X100,
    .rudder_max_roll_deg   = CONTROL_DEFAULT_RUDDER_MAX_ROLL_DEG,
    .height_enabled        = CONTROL_DEFAULT_HEIGHT_ENABLED,
    .joystick_roll_enabled = CONTROL_DEFAULT_JOYSTICK_ROLL_ENABLED,
    .elevon_max_diff_deg   = CONTROL_DEFAULT_ELEVON_MAX_DIFF_DEG,
    .pitch_target_max_deg = CONTROL_DEFAULT_PITCH_TARGET_MAX_DEG,
    .height_target_cm     = CONTROL_DEFAULT_HEIGHT_TARGET_CM,
};

typedef struct {
    float elevon_left_deg;
    float elevon_right_deg;
    bool  armed;
    float pitch_target_deg;
    float roll_target_deg;
} control_output_t;

static control_config_t s_cfg;
static bool s_armed;

static servo_channel_t s_servo_left  = SERVO_CHANNEL_INVALID;
static servo_channel_t s_servo_right = SERVO_CHANNEL_INVALID;

/* Latest joystick axis values from the aux controller (0..100, 50 = centre). */
static volatile uint16_t s_joy_x_pct = 50;
static volatile uint16_t s_joy_y_pct = 50;

/* PID state: integral pre-multiplied by ki (so clamping it bounds the I-term
 * contribution directly), previous measurement for derivative-on-measurement,
 * and a first-call gate so the derivative doesn't spike from prev_input = 0. */
typedef struct {
    float integral;
    float prev_input;
    bool  first;
} pid_state_t;

static pid_state_t s_height_pid;
static pid_state_t s_pitch_pid;
static pid_state_t s_roll_pid;

/* Manual setpoint accumulators (rate-mode joystick). Persist across
 * arm/disarm and config changes — only the joystick mutates them. */
static float s_manual_pitch_target;
static float s_manual_roll_target;

static control_output_t s_last_out;
static void (*s_change_cb)(void);

/* Per-loop iteration timing, accumulated by the control task and reported by
 * the telemetry task. min/max persist for the session; sum/count reset on each
 * report. Cross-task access is unlocked — stats only, races are benign. */
typedef struct {
    uint32_t min_us;
    uint32_t max_us;
    uint64_t sum_us;
    uint32_t count;
} perf_stats_t;

static perf_stats_t s_armed_perf;
static perf_stats_t s_disarmed_perf;

/* PID with derivative-on-measurement (no setpoint kick) and anti-windup
 * (integral contribution clamped to ±out_abs_max, the same bound the output
 * itself is clamped to). Output is clamped to ±out_abs_max. */
static float apply_pid(float setpoint, float input,
                       float kp, float ki, float kd,
                       float dt, float out_abs_max,
                       pid_state_t *st) {
    float error = setpoint - input;

    if (st->first) {
        st->prev_input = input;
        st->integral = 0.0f;
        st->first = false;
    }

    st->integral += ki * error * dt;
    if (st->integral >  out_abs_max) st->integral =  out_abs_max;
    if (st->integral < -out_abs_max) st->integral = -out_abs_max;

    float d_input = (input - st->prev_input) / dt;
    st->prev_input = input;

    float out = kp * error + st->integral - kd * d_input;
    if (out >  out_abs_max) out =  out_abs_max;
    if (out < -out_abs_max) out = -out_abs_max;
    return out;
}

static void reset_integrals(void) {
    s_height_pid = (pid_state_t){.first = true};
    s_pitch_pid  = (pid_state_t){.first = true};
    s_roll_pid   = (pid_state_t){.first = true};
}

/* Pure PID computation — no I/O, no status_ui. Fills *out. */
static void control_compute(int16_t height_cm,
                            float pitch_deg,
                            float roll_deg,
                            float rudder_angle,
                            uint16_t joy_x_pct,
                            uint16_t joy_y_pct,
                            control_output_t *out) {
    const float dt = 0.02f;

    memset(out, 0, sizeof(*out));

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

    const float pitch_target_abs_max = (float)s_cfg.pitch_target_max_deg;
    const float roll_target_abs_max  = (float)s_cfg.rudder_max_roll_deg;
    float max_diff   = (float)s_cfg.elevon_max_diff_deg;
    float max_center = servo_drive_get_effective_range_deg() - max_diff;
    if (max_center < 0.0f) max_center = 0.0f;

    /* Joystick rate-integrates into a pitch trim offset; clamped to the same
     * authority bound as the height PID output. */
    float joy_y_norm = ((float)joy_y_pct / 50.0f) - 1.0f;
    if (joy_y_norm >  1.0f) joy_y_norm =  1.0f;
    if (joy_y_norm < -1.0f) joy_y_norm = -1.0f;
    if (fabsf(joy_y_norm) < 0.05f) joy_y_norm = 0.0f;
    s_manual_pitch_target += joy_y_norm * 20.0f * dt;
    if (s_manual_pitch_target >  pitch_target_abs_max) s_manual_pitch_target =  pitch_target_abs_max;
    if (s_manual_pitch_target < -pitch_target_abs_max) s_manual_pitch_target = -pitch_target_abs_max;

    /* Joystick x axis mirrors the pitch behaviour for a roll trim offset.
     * When disabled, the offset is retained (not zeroed); clamping still runs
     * in case rudder_max_roll_deg was reduced below the current value. */
    if (s_cfg.joystick_roll_enabled) {
        float joy_x_norm = ((float)joy_x_pct / 50.0f) - 1.0f;
        if (joy_x_norm >  1.0f) joy_x_norm =  1.0f;
        if (joy_x_norm < -1.0f) joy_x_norm = -1.0f;
        if (fabsf(joy_x_norm) < 0.05f) joy_x_norm = 0.0f;
        s_manual_roll_target += joy_x_norm * 20.0f * dt;
    }
    if (s_manual_roll_target >  roll_target_abs_max) s_manual_roll_target =  roll_target_abs_max;
    if (s_manual_roll_target < -roll_target_abs_max) s_manual_roll_target = -roll_target_abs_max;

    float pitch_target;
    if (s_cfg.height_enabled) {
        pitch_target = apply_pid((float)s_cfg.height_target_cm, (float)height_cm,
                                 kp_h, ki_h, kd_h, dt, pitch_target_abs_max, &s_height_pid);
        pitch_target += s_manual_pitch_target;
    } else {
        pitch_target = s_manual_pitch_target;
        s_height_pid.first = true;
    }
    if (pitch_target >  pitch_target_abs_max) pitch_target =  pitch_target_abs_max;
    if (pitch_target < -pitch_target_abs_max) pitch_target = -pitch_target_abs_max;

    float elevon_center = apply_pid(pitch_target, pitch_deg,
                                    kp_p, ki_p, kd_p, dt, max_center, &s_pitch_pid);

    float rudder_exp = (float)s_cfg.rudder_exponent_x100 / 100.0f;
    float max_roll = (float)s_cfg.rudder_max_roll_deg;
    float rudder_norm = (max_roll > 0.0f) ? (rudder_angle / max_roll) : 0.0f;
    if (rudder_norm >  1.0f) rudder_norm =  1.0f;
    if (rudder_norm < -1.0f) rudder_norm = -1.0f;
    float abs_norm = fabsf(rudder_norm);
    float roll_target_deg = powf(abs_norm, rudder_exp) * max_roll;
    if (rudder_norm < 0.0f) roll_target_deg = -roll_target_deg;
    roll_target_deg += s_manual_roll_target;
    if (roll_target_deg >  max_roll) roll_target_deg =  max_roll;
    if (roll_target_deg < -max_roll) roll_target_deg = -max_roll;

    float elevon_diff = apply_pid(roll_target_deg, roll_deg,
                                  kp_r, ki_r, kd_r, dt, max_diff, &s_roll_pid);

    out->armed = true;
    out->elevon_left_deg  = elevon_center + elevon_diff;
    out->elevon_right_deg = elevon_center - elevon_diff;
    out->pitch_target_deg = pitch_target;
    out->roll_target_deg  = roll_target_deg;
}

/* ── Hot path: sense → PID → command servos. Highest priority, 50 Hz. ── */
static void ctrl_task(void *arg) {
    (void)arg;
    for (;;) {
        int64_t t0 = esp_timer_get_time();

        if (servo_drive_any_cal_mode() && s_armed) {
            control_disarm();
        }

        float rudder_angle = 0.0f;
        encoder_can_get_angle(&rudder_angle);

        float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
        if (imu_is_ready()) {
            imu_get_pitch_roll_yaw(&pitch, &roll, &yaw);
        }

        int32_t height_cm = 30;
        height_get_cm(&height_cm);

        control_output_t out;
        control_compute((int16_t)height_cm, pitch, roll, rudder_angle,
                        s_joy_x_pct, s_joy_y_pct, &out);

        if (!servo_drive_any_cal_mode()) {
            float l = out.armed ? out.elevon_left_deg  : 0.0f;
            float r = out.armed ? out.elevon_right_deg : 0.0f;
            if (s_servo_left  != SERVO_CHANNEL_INVALID) servo_drive_set_degrees(s_servo_left,  l);
            if (s_servo_right != SERVO_CHANNEL_INVALID) servo_drive_set_degrees(s_servo_right, r);
        }

        s_last_out = out;

        uint32_t dur_us = (uint32_t)(esp_timer_get_time() - t0);
        perf_stats_t *ps = out.armed ? &s_armed_perf : &s_disarmed_perf;
        if (dur_us < ps->min_us) ps->min_us = dur_us;
        if (dur_us > ps->max_us) ps->max_us = dur_us;
        ps->sum_us += dur_us;
        ps->count++;

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void report_perf(perf_stats_t *s, bool is_armed) {
    if (s->count == 0) return;

    uint32_t avg_us = (uint32_t)(s->sum_us / s->count);
    uint32_t avg_hz = avg_us > 0 ? (1000000u / avg_us) : 0;

    status_ui_update("perf" ,
                     "%s iter %"PRIu32"/%"PRIu32"us avg %"PRIu32"Hz",is_armed ? "armed" : "disarmed" ,
                     avg_us, s->max_us, avg_hz);

    can_ctrl_perf_t perf = {
        .iter_avg_us = (avg_us    > 0xFFFFu) ? 0xFFFFu : (uint16_t)avg_us,
        .iter_max_us = (s->max_us > 0xFFFFu) ? 0xFFFFu : (uint16_t)s->max_us,
        .iter_hz     = (avg_hz    > 0xFFFFu) ? 0xFFFFu : (uint16_t)avg_hz,
        .is_armed    = is_armed ? 1u : 0u,
        ._pad        = 0,
    };
    (void)can_tx(CAN_ID_CTRL_PERF, (const uint8_t *)&perf, sizeof(perf));

    s->sum_us = 0;
    s->count  = 0;
}

/* ── Telemetry: CAN status/perf TX + status_ui. Low priority, off the hot path. ── */
static void ctrl_tlm_task(void *arg) {
    (void)arg;
    for (;;) {
        control_output_t out = s_last_out;

        can_ctrl_status_t cs = {
            .height_target_cm     = (uint8_t)(s_cfg.height_target_cm < 0 ? 0 : s_cfg.height_target_cm > 100 ? 100 : s_cfg.height_target_cm),
            .pitch_target_deg_x10 = (int16_t)(out.pitch_target_deg * 10.0f),
            .roll_target_deg_x10  = (int16_t)(out.roll_target_deg  * 10.0f),
            .flags                = s_armed ? 1u : 0u,
        };
        (void)can_tx(CAN_ID_CTRL_STATUS, (const uint8_t *)&cs, sizeof(cs));

        report_perf(s_armed ? &s_armed_perf : &s_disarmed_perf, s_armed);

        float max_diff   = (float)s_cfg.elevon_max_diff_deg;
        float max_angle  = servo_drive_get_effective_range_deg();
        float max_center = max_angle - max_diff;
        if (max_center < 0.0f) max_center = 0.0f;
        float center = (out.elevon_left_deg + out.elevon_right_deg) * 0.5f;
        float diff   = (out.elevon_left_deg - out.elevon_right_deg) * 0.5f;

        int32_t height_cm = 30;
        height_get_cm(&height_cm);

        status_ui_update("Control", "%s target=%d ht=%d L=%.2f R=%.2f",
                         s_armed ? "ARMED" : "DISARM",
                         (int)s_cfg.height_target_cm, (int)height_cm,
                         (double)out.elevon_left_deg, (double)out.elevon_right_deg);
        status_ui_update("Limits", "max=%.2f ctr=%.2f diff=%.2f",
                         (double)max_angle, (double)max_center, (double)max_diff);
        status_ui_update("Elevon", "C=%.2f D=%.2f", (double)center, (double)diff);
        status_ui_update("Setpts", "ht=%d pt=%.2f rl=%.2f",
                         (int)s_cfg.height_target_cm, (double)out.pitch_target_deg, (double)out.roll_target_deg);
        status_ui_update("Joy", "x=%u%% y=%u%%",
                         (unsigned)s_joy_x_pct, (unsigned)s_joy_y_pct);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void joystick_rx_cb(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp) {
    (void)timestamp;
    if (header_id != CAN_ID_JOYSTICK || buffer == NULL) {
        return;
    }
    can_joystick_t joy;
    memcpy(&joy, buffer, sizeof(joy));
    s_joy_x_pct = (joy.x_pct > 100u) ? 100u : joy.x_pct;
    s_joy_y_pct = (joy.y_pct > 100u) ? 100u : joy.y_pct;
}

esp_err_t control_init(servo_channel_t servo_left, servo_channel_t servo_right) {
    if (config_get_blob(CONTROL_NVS_KEY, &s_cfg, sizeof(s_cfg)) != ESP_OK) {
        s_cfg = s_control_defaults;
    }
    s_servo_left  = servo_left;
    s_servo_right = servo_right;
    s_armed = false;
    reset_integrals();
    s_armed_perf    = (perf_stats_t){.min_us = UINT32_MAX};
    s_disarmed_perf = (perf_stats_t){.min_us = UINT32_MAX};

    esp_err_t err = can_register_rx_cb(joystick_rx_cb);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "joystick RX register failed: %s", esp_err_to_name(err));
    }

    if (xTaskCreate(ctrl_task, "ctrl", 8192, NULL, 10, NULL) != pdPASS) {
        ESP_LOGE(TAG, "ctrl task create failed");
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(ctrl_tlm_task, "ctrl_tlm", 4096, NULL, 2, NULL) != pdPASS) {
        ESP_LOGE(TAG, "ctrl_tlm task create failed");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Control initialized, armed=%d target=%d", s_armed, s_cfg.height_target_cm);
    return ESP_OK;
}

void control_apply_cfg(const control_config_t *cfg) {
    if (cfg == NULL) return;
    s_cfg = *cfg;
    if (s_cfg.height_target_cm < 0)  s_cfg.height_target_cm = 0;
    if (s_cfg.height_target_cm > 50) s_cfg.height_target_cm = 50;
    reset_integrals();
    (void)config_set_blob(CONTROL_NVS_KEY, &s_cfg, sizeof(s_cfg));
    if (s_change_cb) s_change_cb();
}

void control_get_cfg(control_config_t *cfg) {
    if (cfg) {
        *cfg = s_cfg;
    }
}

void control_get_defaults(control_config_t *cfg) {
    if (cfg) {
        *cfg = s_control_defaults;
    }
}

void control_register_change_cb(void (*cb)(void)) {
    s_change_cb = cb;
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
        ESP_LOGI(TAG, "Disarmed");
        if (s_change_cb) s_change_cb();
    }
}

void control_get_status(control_status_t *out) {
    if (out == NULL) return;
    out->armed             = s_armed;
    out->elevon_left_deg   = s_last_out.elevon_left_deg;
    out->elevon_right_deg  = s_last_out.elevon_right_deg;
}
