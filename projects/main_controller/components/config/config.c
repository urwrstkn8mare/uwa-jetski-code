#include "config.h"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "config";
static const char *S_NS = "uwa_cfg";

static bool s_initialized;

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
    s_initialized = true;
    return ESP_OK;
}

esp_err_t config_get_blob(const char *key, void *out, size_t size) {
    if (key == NULL || out == NULL || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t h;
    esp_err_t e = nvs_open(S_NS, NVS_READONLY, &h);
    if (e != ESP_OK) {
        return e;
    }
    size_t sz = size;
    e = nvs_get_blob(h, key, out, &sz);
    nvs_close(h);
    if (e == ESP_OK && sz != size) {
        return ESP_ERR_INVALID_SIZE;
    }
    return e;
}

esp_err_t config_set_blob(const char *key, const void *data, size_t size) {
    if (key == NULL || data == NULL || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t h;
    ESP_RETURN_ON_ERROR(nvs_open(S_NS, NVS_READWRITE, &h), TAG, "nvs_open RW failed");
    esp_err_t e = nvs_set_blob(h, key, data, size);
    if (e == ESP_OK) {
        e = nvs_commit(h);
    }
    nvs_close(h);
    return e;
}
