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
    .rudder_exponent_x100 = CONTROL_DEFAULT_RUDDER_EXPONENT_X100,
    .rudder_max_roll_deg  = CONTROL_DEFAULT_RUDDER_MAX_ROLL_DEG,
    .height_enabled       = CONTROL_DEFAULT_HEIGHT_ENABLED,
    .elevon_max_diff_deg  = CONTROL_DEFAULT_ELEVON_MAX_DIFF_DEG,
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
static int16_t s_target_height_cm;

static servo_channel_t s_servo_left  = SERVO_CHANNEL_INVALID;
static servo_channel_t s_servo_right = SERVO_CHANNEL_INVALID;

/* Latest joystick values from the aux controller (0..100, 50 = centre). */
static volatile uint16_t s_joy_bank_pct  = 50;
static volatile uint16_t s_joy_pitch_pct = 50;

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

/* Pure PID computation — no I/O, no status_ui. Fills *out. */
static void control_compute(int16_t height_cm,
                            float pitch_deg,
                            float roll_deg,
                            float rudder_angle,
                            uint16_t joy_pitch_pct,
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

    float max_diff   = (float)s_cfg.elevon_max_diff_deg;
    float max_center = servo_drive_get_effective_range_deg() - max_diff;
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
        control_compute((int16_t)height_cm, pitch, roll, rudder_angle, s_joy_pitch_pct, &out);

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

    status_ui_update(is_armed ? "perf_armed" : "perf_disarmed",
                     "iter %"PRIu32"/%"PRIu32"us avg %"PRIu32"Hz",
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
            .height_target_cm     = (uint8_t)(s_target_height_cm < 0 ? 0 : s_target_height_cm > 100 ? 100 : s_target_height_cm),
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
                         (int)s_target_height_cm, (int)height_cm,
                         (double)out.elevon_left_deg, (double)out.elevon_right_deg);
        status_ui_update("Limits", "max=%.2f ctr=%.2f diff=%.2f",
                         (double)max_angle, (double)max_center, (double)max_diff);
        status_ui_update("Elevon", "C=%.2f D=%.2f", (double)center, (double)diff);
        status_ui_update("Setpts", "ht=%d pt=%.2f rl=%.2f",
                         (int)s_target_height_cm, (double)out.pitch_target_deg, (double)out.roll_target_deg);
        status_ui_update("Joy", "bank=%u%% pitch=%u%%",
                         (unsigned)s_joy_bank_pct, (unsigned)s_joy_pitch_pct);

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
    s_joy_bank_pct  = (joy.bank_pct  > 100u) ? 100u : joy.bank_pct;
    s_joy_pitch_pct = (joy.pitch_pct > 100u) ? 100u : joy.pitch_pct;
}

esp_err_t control_init(servo_channel_t servo_left, servo_channel_t servo_right) {
    if (config_get_blob(CONTROL_NVS_KEY, &s_cfg, sizeof(s_cfg)) != ESP_OK) {
        s_cfg = s_control_defaults;
    }
    s_servo_left  = servo_left;
    s_servo_right = servo_right;
    s_armed = false;
    s_target_height_cm = 0;
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

    ESP_LOGI(TAG, "Control initialized, armed=%d target=%d", s_armed, s_target_height_cm);
    return ESP_OK;
}

void control_apply_cfg(const control_config_t *cfg) {
    if (cfg == NULL) return;
    s_cfg = *cfg;
    reset_integrals();
    (void)config_set_blob(CONTROL_NVS_KEY, &s_cfg, sizeof(s_cfg));
}

void control_get_cfg(control_config_t *cfg) {
    if (cfg) {
        *cfg = s_cfg;
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
        s_target_height_cm = 0;
        ESP_LOGI(TAG, "Disarmed");
        if (s_change_cb) s_change_cb();
    }
}

void control_set_target(int16_t height_cm) {
    if (height_cm < 0) height_cm = 0;
    if (height_cm > 50) height_cm = 50;
    s_target_height_cm = height_cm;
}

void control_get_status(control_status_t *out) {
    if (out == NULL) return;
    out->armed             = s_armed;
    out->target_cm         = s_target_height_cm;
    out->elevon_left_deg   = s_last_out.elevon_left_deg;
    out->elevon_right_deg  = s_last_out.elevon_right_deg;
    out->height_enabled    = s_cfg.height_enabled;
}
