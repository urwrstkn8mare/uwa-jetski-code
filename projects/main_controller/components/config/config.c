#include "config.h"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <math.h>
#include <string.h>

static const char *TAG = "config";
static const char *S_NS = "ctrl_cfg";

static bool s_initialized;

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
    .arm_threshold_pct    = CONTROL_DEFAULT_ARM_THRESHOLD_PCT,
    .disarm_threshold_pct = CONTROL_DEFAULT_DISARM_THRESHOLD_PCT,
    .height_enabled       = CONTROL_DEFAULT_HEIGHT_ENABLED,
    .joy_pitch_max_deg    = CONTROL_DEFAULT_JOY_PITCH_MAX_DEG,
};

static const servo_config_t s_servo_defaults = {
    .channel = {
        {
            .min_pw_us = SERVO_DEFAULT_MIN_PW_US,
            .zero_pw_us = SERVO_DEFAULT_ZERO_PW_US,
            .max_pw_us = SERVO_DEFAULT_MAX_PW_US,
            .min_angle_deg = SERVO_DEFAULT_MIN_ANGLE_DEG,
            .max_angle_deg = SERVO_DEFAULT_MAX_ANGLE_DEG,
        },
        {
            .min_pw_us = SERVO_DEFAULT_MIN_PW_US,
            .zero_pw_us = SERVO_DEFAULT_ZERO_PW_US,
            .max_pw_us = SERVO_DEFAULT_MAX_PW_US,
            .min_angle_deg = SERVO_DEFAULT_MIN_ANGLE_DEG,
            .max_angle_deg = SERVO_DEFAULT_MAX_ANGLE_DEG,
        },
    },
};

static const app_config_t s_defaults = {
    .control = {
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
        .arm_threshold_pct    = CONTROL_DEFAULT_ARM_THRESHOLD_PCT,
        .disarm_threshold_pct = CONTROL_DEFAULT_DISARM_THRESHOLD_PCT,
        .height_enabled       = CONTROL_DEFAULT_HEIGHT_ENABLED,
        .joy_pitch_max_deg    = CONTROL_DEFAULT_JOY_PITCH_MAX_DEG,
    },
    .servo = {
        .channel = {
            {
                .min_pw_us = SERVO_DEFAULT_MIN_PW_US,
                .zero_pw_us = SERVO_DEFAULT_ZERO_PW_US,
                .max_pw_us = SERVO_DEFAULT_MAX_PW_US,
                .min_angle_deg = SERVO_DEFAULT_MIN_ANGLE_DEG,
                .max_angle_deg = SERVO_DEFAULT_MAX_ANGLE_DEG,
            },
            {
                .min_pw_us = SERVO_DEFAULT_MIN_PW_US,
                .zero_pw_us = SERVO_DEFAULT_ZERO_PW_US,
                .max_pw_us = SERVO_DEFAULT_MAX_PW_US,
                .min_angle_deg = SERVO_DEFAULT_MIN_ANGLE_DEG,
                .max_angle_deg = SERVO_DEFAULT_MAX_ANGLE_DEG,
            },
        },
    },
};

static bool servo_calibration_is_valid(const servo_calibration_t *cal) {
    if (cal == NULL) {
        return false;
    }
    if (!isfinite(cal->min_pw_us) || !isfinite(cal->zero_pw_us) || !isfinite(cal->max_pw_us) ||
        !isfinite(cal->min_angle_deg) || !isfinite(cal->max_angle_deg)) {
        return false;
    }
    if (!(cal->min_pw_us < cal->zero_pw_us && cal->zero_pw_us < cal->max_pw_us)) {
        return false;
    }
    if (!(cal->min_angle_deg < 0.0f && cal->max_angle_deg > 0.0f && cal->min_angle_deg < cal->max_angle_deg)) {
        return false;
    }
    return true;
}

static bool servo_config_is_valid(const servo_config_t *cfg) {
    return cfg != NULL &&
           servo_calibration_is_valid(&cfg->channel[0]) &&
           servo_calibration_is_valid(&cfg->channel[1]);
}

esp_err_t config_init(void) {
    if (s_initialized) {
        return ESP_OK;
    }
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS page full / version mismatch — erasing and retrying");
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs_flash_erase failed");
        e = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(e, TAG, "nvs_flash_init failed");

    nvs_handle_t h;
    e = nvs_open(S_NS, NVS_READONLY, &h);
    if (e != ESP_OK) {
        if (e == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "NVS namespace '%s' not found — will use defaults", S_NS);
            s_initialized = true;
            return ESP_OK;
        }
        return e;
    }

    app_config_t stored;
    size_t sz = sizeof(stored);
    e = nvs_get_blob(h, "cfg_blob", &stored, &sz);
    nvs_close(h);

    if (e == ESP_OK && sz == sizeof(stored) && servo_config_is_valid(&stored.servo)) {
        ESP_LOGI(TAG, "Loaded config from NVS");
    } else {
        ESP_LOGW(TAG, "No valid config blob in NVS — will use defaults");
    }

    s_initialized = true;
    return ESP_OK;
}

esp_err_t config_load(app_config_t *out) {
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t h;
    esp_err_t e = nvs_open(S_NS, NVS_READONLY, &h);
    if (e == ESP_ERR_NVS_NOT_FOUND) {
        *out = s_defaults;
        return ESP_OK;
    }
    if (e != ESP_OK) {
        *out = s_defaults;
        return e;
    }

    size_t sz = sizeof(*out);
    e = nvs_get_blob(h, "cfg_blob", out, &sz);
    nvs_close(h);

    if (e != ESP_OK || sz != sizeof(*out) || !servo_config_is_valid(&out->servo)) {
        *out = s_defaults;
    }
    return ESP_OK;
}

esp_err_t config_save(const app_config_t *cfg) {
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t h;
    ESP_RETURN_ON_ERROR(nvs_open(S_NS, NVS_READWRITE, &h), TAG, "nvs_open RW failed");

    esp_err_t e = nvs_set_blob(h, "cfg_blob", cfg, sizeof(*cfg));
    if (e == ESP_OK) {
        e = nvs_commit(h);
    }
    nvs_close(h);

    if (e == ESP_OK) {
        ESP_LOGI(TAG, "Config saved to NVS");
    }
    return e;
}

esp_err_t config_save_servo_cal(int channel_idx, const servo_calibration_t *cal) {
    if (channel_idx < 0 || channel_idx >= 2 || cal == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    app_config_t cfg;
    esp_err_t e = config_load(&cfg);
    if (e != ESP_OK) return e;
    cfg.servo.channel[channel_idx] = *cal;
    return config_save(&cfg);
}

esp_err_t config_save_control_cfg(const control_config_t *cfg) {
    if (cfg == NULL) return ESP_ERR_INVALID_ARG;
    app_config_t app;
    esp_err_t e = config_load(&app);
    if (e != ESP_OK) return e;
    app.control = *cfg;
    return config_save(&app);
}

void config_get_defaults(control_config_t *out) {
    if (out) {
        *out = s_control_defaults;
    }
}

void config_get_servo_defaults(servo_config_t *out) {
    if (out) {
        *out = s_servo_defaults;
    }
}

void config_get_app_defaults(app_config_t *out) {
    if (out) {
        *out = s_defaults;
    }
}
