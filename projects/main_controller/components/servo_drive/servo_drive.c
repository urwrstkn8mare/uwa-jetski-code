#include "servo_drive.h"

#include "can.h"
#include "can_ids.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "status_ui.h"
#include "sdkconfig.h"

#include <math.h>
#include <inttypes.h>
#include <stdio.h>

static const char *TAG = "servo_pwm";

#define SERVO_MAX_INSTANCES 8

typedef struct {
    bool in_use;
    int gpio;
    bool ready;
    bool simulated;
    ledc_channel_t ledc_ch;
    servo_calibration_t cal;
    float cmd_deg;
    bool cal_mode;
} servo_instance_t;

static servo_instance_t s_instances[SERVO_MAX_INSTANCES];
static uint8_t s_ledc_channel_mask;
static bool s_hw_initialized;
static bool s_simulated;
static void (*s_change_cb)(int idx);

static uint32_t s_pwm_freq_hz;
static uint32_t s_pwm_max_duty;
static ledc_mode_t s_speed_mode = LEDC_LOW_SPEED_MODE;
static ledc_timer_t s_timer_num = LEDC_TIMER_0;
static ledc_timer_bit_t s_duty_resolution = LEDC_TIMER_14_BIT;

static const servo_calibration_t s_default_cal = {
    .min_pw_us = 1300.0f,
    .zero_pw_us = 1500.0f,
    .max_pw_us = 1800.0f,
    .min_angle_deg = -8.0f,
    .max_angle_deg = 12.0f,
};

static float clamp_float(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static bool cal_is_valid(const servo_calibration_t *cal) {
    if (cal == NULL) return false;
    if (!isfinite(cal->min_pw_us) || !isfinite(cal->zero_pw_us) || !isfinite(cal->max_pw_us) ||
        !isfinite(cal->min_angle_deg) || !isfinite(cal->max_angle_deg)) return false;
    if (!(cal->min_pw_us < cal->zero_pw_us && cal->zero_pw_us < cal->max_pw_us)) return false;
    if (!(cal->min_angle_deg < 0.0f && cal->max_angle_deg > 0.0f)) return false;
    return true;
}

static float deg_to_pulse_us(const servo_calibration_t *cal, float deg) {
    deg = clamp_float(deg, cal->min_angle_deg, cal->max_angle_deg);
    if (deg <= 0.0f) {
        float span = 0.0f - cal->min_angle_deg;
        if (span <= 0.0f) return cal->zero_pw_us;
        return cal->min_pw_us + (cal->zero_pw_us - cal->min_pw_us) * ((deg - cal->min_angle_deg) / span);
    }
    float span = cal->max_angle_deg;
    if (span <= 0.0f) return cal->zero_pw_us;
    return cal->zero_pw_us + (cal->max_pw_us - cal->zero_pw_us) * (deg / span);
}

static uint32_t pulse_us_to_duty(uint32_t pulse_us) {
    if (s_pwm_freq_hz == 0u || s_pwm_max_duty == 0u) return 0u;
    uint64_t duty = (uint64_t)pulse_us * (uint64_t)s_pwm_freq_hz * (uint64_t)s_pwm_max_duty;
    duty /= 1000000ULL;
    if (duty > s_pwm_max_duty) duty = s_pwm_max_duty;
    return (uint32_t)duty;
}

static int first_free_ledc_channel(void) {
    for (int i = 0; i < 8; i++) {
        if ((s_ledc_channel_mask & (1u << i)) == 0) return i;
    }
    return -1;
}

static void servo_drive_update_status(void) {
    int hw = 0, sim = 0, cal = 0, total = 0;
    float d0 = 0.0f, d1 = 0.0f;
    for (int i = 0; i < SERVO_MAX_INSTANCES; i++) {
        if (!s_instances[i].in_use) continue;
        total++;
        if (s_instances[i].cal_mode) cal++;
        else if (s_instances[i].simulated) sim++;
        else hw++;
        if (i == 0) d0 = s_instances[i].cmd_deg;
        if (i == 1) d1 = s_instances[i].cmd_deg;
    }
    if (total == 0) {
        status_ui_update("Servo", "no servos");
        return;
    }
    const char *mode = cal > 0 ? "CAL" : (hw > 0 ? "HW" : "SIM");
    status_ui_update("Servo", "%s %d/%d L=%.1f R=%.1f", mode, hw + sim, total, d0, d1);
}

static void servo_drive_push_instance(int idx) {
    if (idx < 0 || idx >= SERVO_MAX_INSTANCES || !s_instances[idx].in_use) return;

    uint32_t pulse_us = (uint32_t)lroundf(deg_to_pulse_us(&s_instances[idx].cal, s_instances[idx].cmd_deg));

    if (s_instances[idx].simulated) {
        servo_drive_update_status();
        can_servo_pos_t sp = {
            .channel = (uint8_t)idx,
            .deg     = s_instances[idx].cmd_deg,
        };
        (void)can_tx(CAN_ID_SERVO_POS, (const uint8_t *)&sp, sizeof(sp));
        if (s_change_cb) s_change_cb(idx);
        return;
    }

    uint32_t duty = pulse_us_to_duty(pulse_us);
    (void)ledc_set_duty(s_speed_mode, s_instances[idx].ledc_ch, duty);
    (void)ledc_update_duty(s_speed_mode, s_instances[idx].ledc_ch);
    servo_drive_update_status();
    can_servo_pos_t sp = {
        .channel = (uint8_t)idx,
        .deg     = s_instances[idx].cmd_deg,
    };
    (void)can_tx(CAN_ID_SERVO_POS, (const uint8_t *)&sp, sizeof(sp));
    if (s_change_cb) s_change_cb(idx);
}

esp_err_t servo_drive_init_hw(void) {
    memset(s_instances, 0, sizeof(s_instances));
    s_ledc_channel_mask = 0;
    s_hw_initialized = false;
    s_simulated = false;
    s_speed_mode = LEDC_LOW_SPEED_MODE;
    s_timer_num = LEDC_TIMER_0;
    s_duty_resolution = LEDC_TIMER_14_BIT;
    s_pwm_freq_hz = 50u;
    s_pwm_max_duty = (1u << LEDC_TIMER_14_BIT) - 1u;

#if CONFIG_SERVO_SKIP_HW
    ESP_LOGW(TAG, "Servo PWM disabled by Kconfig — simulated mode");
    s_simulated = true;
    s_hw_initialized = true;
    return ESP_OK;
#endif

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t e = ledc_timer_config(&timer);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "timer config failed (%s) — simulated mode", esp_err_to_name(e));
        s_simulated = true;
        s_hw_initialized = true;
        return ESP_OK;
    }

    s_pwm_freq_hz = ledc_get_freq(s_speed_mode, s_timer_num);
    if (s_pwm_freq_hz == 0u) s_pwm_freq_hz = 50u;

    s_hw_initialized = true;
    ESP_LOGI(TAG, "LEDC timer configured: %" PRIu32 "Hz, %d-bit", s_pwm_freq_hz, LEDC_TIMER_14_BIT);
    return ESP_OK;
}

servo_channel_t servo_drive_open(int gpio) {
    if (!s_hw_initialized) return SERVO_CHANNEL_INVALID;

    int slot = -1;
    for (int i = 0; i < SERVO_MAX_INSTANCES; i++) {
        if (!s_instances[i].in_use) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        ESP_LOGW(TAG, "no free servo slots");
        return SERVO_CHANNEL_INVALID;
    }

    int ledc_ch_num = first_free_ledc_channel();
    if (ledc_ch_num < 0 && !s_simulated) {
        ESP_LOGW(TAG, "no free LEDC channels");
        return SERVO_CHANNEL_INVALID;
    }

    s_instances[slot].in_use = true;
    s_instances[slot].gpio = gpio;
    s_instances[slot].cmd_deg = 0.0f;
    s_instances[slot].cal_mode = false;
    s_instances[slot].cal = s_default_cal;
    s_instances[slot].simulated = s_simulated;
    s_instances[slot].ready = false;

    if (s_simulated) {
        s_instances[slot].ledc_ch = LEDC_CHANNEL_MAX;
        s_instances[slot].ready = true;
        ESP_LOGI(TAG, "servo open: slot=%d gpio=%d [SIM]", slot, gpio);
        return (servo_channel_t)slot;
    }

    ledc_channel_t ledc_channel = (ledc_channel_t)ledc_ch_num;
    s_instances[slot].ledc_ch = ledc_channel;

    uint32_t init_pulse = (uint32_t)lroundf(deg_to_pulse_us(&s_default_cal, 0.0f));
    uint32_t init_duty = pulse_us_to_duty(init_pulse);

    ledc_channel_config_t ch = {
        .gpio_num = (int)gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = ledc_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = init_duty,
        .hpoint = 0,
    };
    esp_err_t e = ledc_channel_config(&ch);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "ch config failed gpio=%d (%s) — SIM", gpio, esp_err_to_name(e));
        s_instances[slot].ledc_ch = LEDC_CHANNEL_MAX;
        s_instances[slot].simulated = true;
        s_instances[slot].ready = true;
        s_instances[slot].cal_mode = false;
        ESP_LOGI(TAG, "servo open: slot=%d gpio=%d [SIM]", slot, gpio);
        return (servo_channel_t)slot;
    }

    s_instances[slot].ready = true;
    s_ledc_channel_mask |= (1u << ledc_ch_num);
    ESP_LOGI(TAG, "servo open: slot=%d gpio=%d ledc_ch=%d", slot, gpio, ledc_ch_num);
    return (servo_channel_t)slot;
}

esp_err_t servo_drive_close(servo_channel_t h) {
    if (h >= SERVO_MAX_INSTANCES || !s_instances[h].in_use) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_instances[h].simulated && s_instances[h].ledc_ch != LEDC_CHANNEL_MAX) {
        uint8_t mask = (uint8_t)(1u << s_instances[h].ledc_ch);
        s_ledc_channel_mask &= ~mask;
    }

    s_instances[h].in_use = false;
    s_instances[h].ready = false;
    s_instances[h].ledc_ch = LEDC_CHANNEL_MAX;
    ESP_LOGI(TAG, "servo close: slot=%d", h);
    return ESP_OK;
}

void servo_drive_set_degrees(servo_channel_t h, float deg) {
    if (h >= SERVO_MAX_INSTANCES || !s_instances[h].in_use) return;
    s_instances[h].cmd_deg = clamp_float(deg,
                                          s_instances[h].cal.min_angle_deg,
                                          s_instances[h].cal.max_angle_deg);
    servo_drive_push_instance((int)h);
}

void servo_drive_get_commanded_degrees(servo_channel_t h, float *out_deg) {
    if (out_deg) {
        *out_deg = 0.0f;
    }
    if (h >= SERVO_MAX_INSTANCES || !s_instances[h].in_use) return;
    if (out_deg) *out_deg = s_instances[h].cmd_deg;
}

void servo_drive_set_cal_mode(servo_channel_t h, bool on) {
    if (h >= SERVO_MAX_INSTANCES || !s_instances[h].in_use) return;
    s_instances[h].cal_mode = on;
    servo_drive_update_status();
    if (s_change_cb) s_change_cb((int)h);
}

bool servo_drive_is_cal_mode(servo_channel_t h) {
    if (h >= SERVO_MAX_INSTANCES || !s_instances[h].in_use) return false;
    return s_instances[h].cal_mode;
}

void servo_drive_apply_cal(servo_channel_t h, const servo_calibration_t *cal) {
    if (h >= SERVO_MAX_INSTANCES || !s_instances[h].in_use) return;
    if (cal == NULL || !cal_is_valid(cal)) return;
    s_instances[h].cal = *cal;
    servo_drive_push_instance((int)h);
}

void servo_drive_get_cal(servo_channel_t h, servo_calibration_t *out_cal) {
    if (out_cal) memset(out_cal, 0, sizeof(*out_cal));
    if (h >= SERVO_MAX_INSTANCES || !s_instances[h].in_use) return;
    if (out_cal) *out_cal = s_instances[h].cal;
}

bool servo_drive_any_cal_mode(void) {
    for (int i = 0; i < SERVO_MAX_INSTANCES; i++) {
        if (s_instances[i].in_use && s_instances[i].cal_mode) return true;
    }
    return false;
}

bool servo_drive_all_ready(void) {
    for (int i = 0; i < SERVO_MAX_INSTANCES; i++) {
        if (s_instances[i].in_use && !s_instances[i].ready) return false;
    }
    return true;
}

int servo_drive_get_count(void) {
    int count = 0;
    for (int i = 0; i < SERVO_MAX_INSTANCES; i++) {
        if (s_instances[i].in_use) count++;
    }
    return count;
}

bool servo_drive_get_info_by_index(int idx, servo_info_t *out_info) {
    if (out_info == NULL) return false;
    memset(out_info, 0, sizeof(*out_info));
    if (idx < 0 || idx >= SERVO_MAX_INSTANCES) return false;
    if (!s_instances[idx].in_use) return false;
    out_info->in_use = true;
    out_info->gpio = s_instances[idx].gpio;
    out_info->ready = s_instances[idx].ready;
    out_info->simulated = s_instances[idx].simulated;
    out_info->cal_mode = s_instances[idx].cal_mode;
    out_info->cmd_deg = s_instances[idx].cmd_deg;
    out_info->cal = s_instances[idx].cal;
    return true;
}

void servo_drive_register_change_cb(void (*cb)(int idx)) {
    s_change_cb = cb;
}