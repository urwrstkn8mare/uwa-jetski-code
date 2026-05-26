#include "encoder_can.h"

#include "can.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"
#include "status_ui.h"

#include <math.h>
#include <string.h>

static const char *TAG = "encoder_can";

/* Briterencoder CAN command codes (inner protocol) */
#define ENCODER_CMD_VALUE    0x01u
#define ENCODER_CMD_SET_MODE 0x04u
#define ENCODER_CMD_SET_TIME 0x05u
#define ENCODER_MODE_AUTO    0xAAu

/* Frame byte offsets in the 8-byte CAN data field */
#define FRAME_OFF_LEN     0
#define FRAME_OFF_DEV     1
#define FRAME_OFF_CMD     2
#define FRAME_OFF_DATA0   3

/* Expected length byte for a value response frame */
#define FRAME_LEN_VALUE   0x07u

static SemaphoreHandle_t s_mx;
static float    s_angle_deg;
static bool     s_have;
static int64_t  s_last_us;

static void ensure_mutex(void) {
    if (s_mx == NULL) {
        s_mx = xSemaphoreCreateMutex();
    }
}

esp_err_t encoder_can_init(void) {
    ensure_mutex();

    /* Set auto-return period to CONFIG_ENCODER_AUTO_PERIOD_MS milliseconds.
     * Command 0x05: [LEN=5][DevID][0x05][period_lo][period_hi] */
    uint16_t period_ms = (uint16_t)CONFIG_ENCODER_AUTO_PERIOD_MS;
    uint8_t set_period[5] = {
        0x05,
        (uint8_t)CONFIG_ENCODER_DEVICE_ID,
        ENCODER_CMD_SET_TIME,
        (uint8_t)(period_ms & 0xFFu),
        (uint8_t)((period_ms >> 8) & 0xFFu),
    };
    esp_err_t err = can_tx((uint32_t)CONFIG_ENCODER_DEVICE_ID, set_period, sizeof(set_period));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "set-period TX failed: %s", esp_err_to_name(err));
    }

    /* Set mode to auto-return encoder value (0xAA).
     * Command 0x04: [LEN=4][DevID][0x04][0xAA] */
    uint8_t set_mode[4] = {
        0x04,
        (uint8_t)CONFIG_ENCODER_DEVICE_ID,
        ENCODER_CMD_SET_MODE,
        ENCODER_MODE_AUTO,
    };
    err = can_tx((uint32_t)CONFIG_ENCODER_DEVICE_ID, set_mode, sizeof(set_mode));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "set-mode TX failed: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "Encoder configured: id=%d res=%d period=%dms max_angle=%d°",
             CONFIG_ENCODER_DEVICE_ID, CONFIG_ENCODER_RESOLUTION,
             CONFIG_ENCODER_AUTO_PERIOD_MS, CONFIG_ENCODER_MAX_ANGLE_DEG);
    return ESP_OK;
}

void encoder_can_on_rx(const uint8_t buffer[8], uint32_t header_id) {
    if ((uint32_t)CONFIG_ENCODER_DEVICE_ID != header_id) {
        return;
    }
    if (buffer == NULL) {
        return;
    }
    /* Validate inner frame: must be a value response */
    if (buffer[FRAME_OFF_LEN] != FRAME_LEN_VALUE) {
        return;
    }
    if (buffer[FRAME_OFF_CMD] != ENCODER_CMD_VALUE) {
        return;
    }

    uint32_t raw;
    memcpy(&raw, &buffer[FRAME_OFF_DATA0], sizeof(raw));

    /* Convert raw count to signed angle using wrap-around at half-resolution.
     * After the user zeros the encoder, values close to resolution wrap to
     * negative (counterclockwise). */
    const uint32_t res = (uint32_t)CONFIG_ENCODER_RESOLUTION;
    int32_t signed_raw = (raw > res / 2u) ? (int32_t)raw - (int32_t)res : (int32_t)raw;
    float angle = (float)signed_raw * 360.0f / (float)res;

    ensure_mutex();
    xSemaphoreTake(s_mx, portMAX_DELAY);
    s_angle_deg = angle;
    s_have      = true;
    s_last_us   = esp_timer_get_time();
    xSemaphoreGive(s_mx);

    status_ui_update("Encoder", "%.1f°", (double)angle);
}

bool encoder_can_is_fresh(uint32_t max_age_ms, float *angle_out) {
    ensure_mutex();

    xSemaphoreTake(s_mx, portMAX_DELAY);
    const bool     have  = s_have;
    const float    angle = s_angle_deg;
    const int64_t  last  = s_last_us;
    xSemaphoreGive(s_mx);

    if (!have) {
        return false;
    }
    const int64_t max_dt = (int64_t)(max_age_ms ? max_age_ms : 1u) * 1000LL;
    if ((esp_timer_get_time() - last) > max_dt) {
        return false;
    }
    if (angle_out != NULL) {
        *angle_out = angle;
    }
    return true;
}
