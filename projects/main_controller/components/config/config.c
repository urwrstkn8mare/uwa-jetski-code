#include "config.h"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <string.h>

static const char *TAG = "config";
static const char *S_NS = "ctrl_cfg";

static bool s_initialized;

static const control_config_t s_defaults = {
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
};

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

    control_config_t stored;
    size_t sz = sizeof(stored);
    e = nvs_get_blob(h, "cfg_blob", &stored, &sz);
    nvs_close(h);

    if (e == ESP_OK && sz == sizeof(stored)) {
        ESP_LOGI(TAG, "Loaded config from NVS");
    } else {
        ESP_LOGW(TAG, "No valid config blob in NVS — will use defaults");
    }

    s_initialized = true;
    return ESP_OK;
}

esp_err_t config_load(control_config_t *out) {
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

    if (e != ESP_OK || sz != sizeof(*out)) {
        *out = s_defaults;
    }
    return ESP_OK;
}

esp_err_t config_save(const control_config_t *cfg) {
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

void config_get_defaults(control_config_t *out) {
    if (out) {
        *out = s_defaults;
    }
}
