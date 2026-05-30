#include "encoder_can.h"

#include "can.h"
#include "esp_log.h"
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

/* Auto-feedback frames arrive on 0x180 + device_id, not on device_id */
#define ENCODER_AUTO_FEEDBACK_BASE 0x180u

static volatile float s_angle_deg;
static volatile bool  s_have;

static void encoder_rx_cb(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp);

esp_err_t encoder_can_init(bool configure) {
    esp_err_t reg = can_register_rx_cb(encoder_rx_cb);
    if (reg != ESP_OK) {
        return reg;
    }

    if (!configure) {
        return ESP_OK;
    }

    /* Set mode to auto-return encoder value (0xAA) — must come before period.
     * Command 0x04: [LEN=4][DevID][0x04][0xAA] */
    uint8_t set_mode[4] = {
        0x04,
        (uint8_t)CONFIG_ENCODER_DEVICE_ID,
        ENCODER_CMD_SET_MODE,
        ENCODER_MODE_AUTO,
    };
    esp_err_t err = can_tx((uint32_t)CONFIG_ENCODER_DEVICE_ID, set_mode, sizeof(set_mode));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "set-mode TX failed: %s", esp_err_to_name(err));
    }

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
    err = can_tx((uint32_t)CONFIG_ENCODER_DEVICE_ID, set_period, sizeof(set_period));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "set-period TX failed: %s", esp_err_to_name(err));
    }

    /* Request an immediate first reading before auto-feedback kicks in.
     * Command 0x04: [LEN=4][DevID][0x01][0x00] */
    uint8_t read_val[4] = {
        0x04,
        (uint8_t)CONFIG_ENCODER_DEVICE_ID,
        ENCODER_CMD_VALUE,
        0x00,
    };
    err = can_tx((uint32_t)CONFIG_ENCODER_DEVICE_ID, read_val, sizeof(read_val));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "read-value TX failed: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "Encoder configured: id=%d res=%d period=%dms max_angle=%d°",
             CONFIG_ENCODER_DEVICE_ID, CONFIG_ENCODER_RESOLUTION,
             CONFIG_ENCODER_AUTO_PERIOD_MS, CONFIG_ENCODER_MAX_ANGLE_DEG);
    return ESP_OK;
}

static void encoder_rx_cb(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp) {
    (void)timestamp;
    if (buffer == NULL) {
        return;
    }

    uint32_t raw;

    if (header_id == (uint32_t)(ENCODER_AUTO_FEEDBACK_BASE + CONFIG_ENCODER_DEVICE_ID)) {
        /* Auto-feedback frame: raw value is the first 4 bytes, little-endian.
         * No header bytes — the payload IS the count. */
        memcpy(&raw, buffer, sizeof(raw));
    } else if (header_id == (uint32_t)CONFIG_ENCODER_DEVICE_ID) {
        /* Response to an explicit read command: [LEN=7][DevID][0x01][val 4B LE] */
        if (buffer[FRAME_OFF_LEN] != FRAME_LEN_VALUE) {
            return;
        }
        if (buffer[FRAME_OFF_CMD] != ENCODER_CMD_VALUE) {
            return;
        }
        memcpy(&raw, &buffer[FRAME_OFF_DATA0], sizeof(raw));
    } else {
        return;
    }

    /* Convert raw count to signed angle using wrap-around at half-resolution.
     * After the user zeros the encoder, values close to resolution wrap to
     * negative (counterclockwise). */
    const uint32_t res = (uint32_t)CONFIG_ENCODER_RESOLUTION;
    int32_t signed_raw = (raw > res / 2u) ? (int32_t)raw - (int32_t)res : (int32_t)raw;
    float angle = (float)signed_raw * 360.0f / (float)res;

    s_angle_deg = angle;
    s_have      = true;

    status_ui_update("Encoder", "%.2f°", (double)angle);
}

bool encoder_can_get_angle(float *angle_out) {
    if (!s_have) {
        return false;
    }
    if (angle_out != NULL) {
        float clamped = s_angle_deg;
        if      (clamped >  20.0f) clamped =  20.0f;
        else if (clamped < -20.0f) clamped = -20.0f;
        *angle_out = clamped + 3.52f;
    }
    return true;
}
