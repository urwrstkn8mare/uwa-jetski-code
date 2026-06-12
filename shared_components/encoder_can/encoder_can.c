#include "encoder_can.h"

#include "can.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "status_ui.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>

static const char *TAG = "encoder_can";

/* Briterencoder CAN command codes (inner protocol) */
#define ENCODER_CMD_VALUE    0x01u
#define ENCODER_CMD_SET_MODE 0x04u
#define ENCODER_CMD_SET_TIME 0x05u
#define ENCODER_CMD_SET_ZERO 0x06u
#define ENCODER_MODE_AUTO    0xAAu

/* Frame byte offsets in the 8-byte CAN data field */
#define FRAME_OFF_LEN     0
#define FRAME_OFF_DEV     1
#define FRAME_OFF_CMD     2
#define FRAME_OFF_DATA0   3

/* Expected length byte for a value response frame */
#define FRAME_LEN_VALUE   0x07u
#define FRAME_LEN_ACK     0x04u

/* Auto-feedback frames arrive on 0x180 + device_id, not on device_id */
#define ENCODER_AUTO_FEEDBACK_BASE 0x180u

/* CANopen SDO COB-ID bases, used only to probe whether the encoder is
 * actually the CANopen variant (which ignores the CAN2.0B command set). */
#define CANOPEN_SDO_TX_BASE 0x600u /* host -> encoder */
#define CANOPEN_SDO_RX_BASE 0x580u /* encoder -> host */

#define ENCODER_ACK_DONE_BIT BIT0
#define ENCODER_SDO_RESP_BIT BIT1

static volatile float s_angle_deg;
static volatile bool  s_have;
static volatile uint8_t s_pending_ack_cmd;
static volatile uint8_t s_last_ack_cmd;
static volatile uint8_t s_last_ack_status;
static uint8_t s_last_sdo_resp[8];
static EventGroupHandle_t s_ack_events;

static void encoder_rx_cb(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp);
static esp_err_t encoder_send_and_wait_ack(uint8_t cmd, const uint8_t *data, uint8_t len,
                                           uint8_t *status_out, TickType_t timeout_ticks);

esp_err_t encoder_can_init(bool configure) {
    if (s_ack_events == NULL) {
        s_ack_events = xEventGroupCreate();
        if (s_ack_events == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

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
    } else if (header_id == (uint32_t)(CANOPEN_SDO_RX_BASE + CONFIG_ENCODER_DEVICE_ID)) {
        /* SDO response: the encoder is the CANopen variant. */
        memcpy(s_last_sdo_resp, buffer, sizeof(s_last_sdo_resp));
        ESP_LOGI(TAG, "SDO response: %02x %02x %02x %02x %02x %02x %02x %02x",
                 buffer[0], buffer[1], buffer[2], buffer[3],
                 buffer[4], buffer[5], buffer[6], buffer[7]);
        if (s_ack_events != NULL) {
            xEventGroupSetBits(s_ack_events, ENCODER_SDO_RESP_BIT);
        }
        return;
    } else if (header_id == (uint32_t)CONFIG_ENCODER_DEVICE_ID) {
        /* Acknowledgement to a write command: [LEN=4][DevID][CMD][status] */
        if (buffer[FRAME_OFF_LEN] == FRAME_LEN_ACK &&
            buffer[FRAME_OFF_DEV] == (uint8_t)CONFIG_ENCODER_DEVICE_ID &&
            buffer[FRAME_OFF_CMD] != ENCODER_CMD_VALUE) {
            uint8_t cmd = buffer[FRAME_OFF_CMD];
            uint8_t status = buffer[FRAME_OFF_DATA0];
            s_last_ack_cmd = cmd;
            s_last_ack_status = status;
            ESP_LOGI(TAG, "ack cmd=0x%02x status=0x%02x", cmd, status);
            if (s_ack_events != NULL && s_pending_ack_cmd == cmd) {
                xEventGroupSetBits(s_ack_events, ENCODER_ACK_DONE_BIT);
            }
            return;
        }

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
        *angle_out = clamped;
    }
    return true;
}

static esp_err_t encoder_send_and_wait_ack(uint8_t cmd, const uint8_t *data, uint8_t len,
                                           uint8_t *status_out, TickType_t timeout_ticks) {
    if (s_ack_events == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == NULL || len < 4 || len > 8) {
        return ESP_ERR_INVALID_ARG;
    }

    xEventGroupClearBits(s_ack_events, ENCODER_ACK_DONE_BIT);
    s_pending_ack_cmd = cmd;
    s_last_ack_cmd = 0;
    s_last_ack_status = 0xFFu;

    esp_err_t err = can_tx((uint32_t)CONFIG_ENCODER_DEVICE_ID, data, len);
    if (err != ESP_OK) {
        s_pending_ack_cmd = 0;
        return err;
    }

    EventBits_t bits = xEventGroupWaitBits(s_ack_events, ENCODER_ACK_DONE_BIT,
                                           pdTRUE, pdFALSE, timeout_ticks);
    s_pending_ack_cmd = 0;
    if ((bits & ENCODER_ACK_DONE_BIT) == 0) {
        ESP_LOGW(TAG, "ack timeout cmd=0x%02x", cmd);
        return ESP_ERR_TIMEOUT;
    }

    uint8_t status = s_last_ack_status;
    if (status_out != NULL) {
        *status_out = status;
    }
    /* Mode-set (0x04) acks echo the written value (e.g. 0xAA) instead of a
     * 0x00 success status, so an echo of the sent data byte is also success. */
    if (status != 0x00u && status != data[FRAME_OFF_DATA0]) {
        ESP_LOGW(TAG, "ack error cmd=0x%02x status=0x%02x", cmd, status);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* Expedited CANopen SDO download (write) of an n-byte value to index:sub.
 * Returns ESP_OK on a 0x60 response, ESP_FAIL with *abort_out filled on a
 * 0x80 abort, ESP_ERR_TIMEOUT if the encoder does not answer SDO at all. */
static esp_err_t encoder_sdo_write(uint16_t index, uint8_t sub, uint32_t value,
                                   uint8_t nbytes, uint32_t *abort_out,
                                   TickType_t timeout_ticks) {
    if (s_ack_events == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (nbytes != 1 && nbytes != 2 && nbytes != 4) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Command specifier: expedited download, size indicated. */
    uint8_t req[8] = {
        (uint8_t)(0x23u | ((4u - nbytes) << 2)),
        (uint8_t)(index & 0xFFu),
        (uint8_t)(index >> 8),
        sub,
        (uint8_t)(value & 0xFFu),
        (uint8_t)((value >> 8) & 0xFFu),
        (uint8_t)((value >> 16) & 0xFFu),
        (uint8_t)((value >> 24) & 0xFFu),
    };
    xEventGroupClearBits(s_ack_events, ENCODER_SDO_RESP_BIT);
    esp_err_t err = can_tx(CANOPEN_SDO_TX_BASE + (uint32_t)CONFIG_ENCODER_DEVICE_ID,
                           req, sizeof(req));
    if (err != ESP_OK) {
        return err;
    }
    EventBits_t bits = xEventGroupWaitBits(s_ack_events, ENCODER_SDO_RESP_BIT,
                                           pdTRUE, pdFALSE, timeout_ticks);
    if ((bits & ENCODER_SDO_RESP_BIT) == 0) {
        ESP_LOGW(TAG, "SDO write %04x:%02x: no response", index, sub);
        return ESP_ERR_TIMEOUT;
    }
    if (s_last_sdo_resp[1] != (uint8_t)(index & 0xFFu) ||
        s_last_sdo_resp[2] != (uint8_t)(index >> 8) ||
        s_last_sdo_resp[3] != sub) {
        ESP_LOGW(TAG, "SDO write %04x:%02x: response for wrong object", index, sub);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (s_last_sdo_resp[0] == 0x60u) {
        return ESP_OK;
    }
    if (s_last_sdo_resp[0] == 0x80u) {
        uint32_t abort;
        memcpy(&abort, &s_last_sdo_resp[4], sizeof(abort));
        if (abort_out != NULL) {
            *abort_out = abort;
        }
        ESP_LOGW(TAG, "SDO write %04x:%02x: abort 0x%08" PRIx32, index, sub, abort);
        return ESP_FAIL;
    }
    return ESP_ERR_INVALID_RESPONSE;
}

/* CANopen objects used for zeroing (CiA 406 encoder profile; the fitted
 * encoder reports device type 0x00010196 = single-turn, profile 406). */
#define CANOPEN_OBJ_PRESET     0x6003u /* preset value: write 0 -> position 0 */
#define CANOPEN_OBJ_STORE      0x1010u /* store parameters */
#define CANOPEN_STORE_SUB_ALL  0x01u
#define CANOPEN_STORE_MAGIC    0x65766173u /* "save" little-endian */

/* SDO aborts indicating a data-size mismatch, worth retrying as 16-bit */
#define SDO_ABORT_TYPE_MISMATCH 0x06070010u
#define SDO_ABORT_LEN_HIGH      0x06070012u
#define SDO_ABORT_LEN_LOW       0x06070013u

esp_err_t encoder_can_zero(encoder_zero_report_t *report_out) {
    /* The fitted encoder is the CANopen variant: it answers SDO requests on
     * 0x580/0x600 + node id and silently ignores the CAN2.0B parameter writes
     * from the Briter "CANbus" manual (only the 0x01 read is answered).
     * Zero it by writing 0 to the CiA 406 preset object 0x6003 and persist
     * with a 0x1010:01 store. The CAN2.0B command sequence (0x04/0x06) is
     * kept as a fallback in case a non-CANopen encoder is ever fitted.
     */
    uint8_t set_query_mode[4] = {
        0x04,
        (uint8_t)CONFIG_ENCODER_DEVICE_ID,
        ENCODER_CMD_SET_MODE,
        0x00,
    };
    uint8_t set_zero[4] = {
        0x04,
        (uint8_t)CONFIG_ENCODER_DEVICE_ID,
        ENCODER_CMD_SET_ZERO,
        0x00,
    };
    uint8_t set_auto_mode[4] = {
        0x04,
        (uint8_t)CONFIG_ENCODER_DEVICE_ID,
        ENCODER_CMD_SET_MODE,
        ENCODER_MODE_AUTO,
    };
    uint8_t read_val[4] = {
        0x04,
        (uint8_t)CONFIG_ENCODER_DEVICE_ID,
        ENCODER_CMD_VALUE,
        0x00,
    };
    encoder_zero_report_t rep = {
        .sdo_preset_err     = ESP_ERR_NOT_FINISHED, .sdo_preset_abort    = 0,
        .sdo_save_err       = ESP_ERR_NOT_FINISHED, .sdo_save_abort      = 0,
        .mode_pause_err     = ESP_ERR_NOT_FINISHED, .mode_pause_status   = 0xFFu,
        .zero_err           = ESP_ERR_NOT_FINISHED, .zero_status         = 0xFFu,
        .mode_restore_err   = ESP_ERR_NOT_FINISHED, .mode_restore_status = 0xFFu,
    };
    esp_err_t read_err = ESP_OK;

    rep.sdo_preset_err = encoder_sdo_write(CANOPEN_OBJ_PRESET, 0x00, 0, 4,
                                           &rep.sdo_preset_abort, pdMS_TO_TICKS(500));
    if (rep.sdo_preset_err == ESP_FAIL &&
        (rep.sdo_preset_abort == SDO_ABORT_TYPE_MISMATCH ||
         rep.sdo_preset_abort == SDO_ABORT_LEN_HIGH ||
         rep.sdo_preset_abort == SDO_ABORT_LEN_LOW)) {
        /* Some encoders implement the preset object as Unsigned16. */
        rep.sdo_preset_err = encoder_sdo_write(CANOPEN_OBJ_PRESET, 0x00, 0, 2,
                                               &rep.sdo_preset_abort,
                                               pdMS_TO_TICKS(500));
    }
    if (rep.sdo_preset_err == ESP_OK) {
        /* Persist across power cycles; EEPROM writes can be slow. */
        rep.sdo_save_err = encoder_sdo_write(CANOPEN_OBJ_STORE, CANOPEN_STORE_SUB_ALL,
                                             CANOPEN_STORE_MAGIC, 4,
                                             &rep.sdo_save_abort, pdMS_TO_TICKS(1000));
        if (rep.sdo_save_err != ESP_OK) {
            ESP_LOGW(TAG, "zeroed but store failed: %s",
                     esp_err_to_name(rep.sdo_save_err));
        }
        goto out;
    }
    if (rep.sdo_preset_err != ESP_ERR_TIMEOUT) {
        /* The encoder speaks SDO but refused the preset — report, don't fall
         * through to a protocol it ignores anyway. */
        goto out;
    }

    rep.mode_pause_err = encoder_send_and_wait_ack(ENCODER_CMD_SET_MODE, set_query_mode,
                                                   sizeof(set_query_mode),
                                                   &rep.mode_pause_status,
                                                   pdMS_TO_TICKS(250));
    if (rep.mode_pause_err != ESP_OK) {
        ESP_LOGW(TAG, "zero pre-mode failed: %s status=0x%02x",
                 esp_err_to_name(rep.mode_pause_err), rep.mode_pause_status);
        goto out;
    }

    vTaskDelay(pdMS_TO_TICKS(CONFIG_ENCODER_AUTO_PERIOD_MS + 10));

    rep.zero_err = encoder_send_and_wait_ack(ENCODER_CMD_SET_ZERO, set_zero, sizeof(set_zero),
                                             &rep.zero_status, pdMS_TO_TICKS(500));
    if (rep.zero_err != ESP_OK) {
        ESP_LOGW(TAG, "zero failed: %s status=0x%02x",
                 esp_err_to_name(rep.zero_err), rep.zero_status);
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    rep.mode_restore_err = encoder_send_and_wait_ack(ENCODER_CMD_SET_MODE, set_auto_mode,
                                                     sizeof(set_auto_mode),
                                                     &rep.mode_restore_status,
                                                     pdMS_TO_TICKS(250));
    if (rep.mode_restore_err != ESP_OK) {
        ESP_LOGW(TAG, "zero restore-mode failed: %s status=0x%02x",
                 esp_err_to_name(rep.mode_restore_err), rep.mode_restore_status);
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    read_err = can_tx((uint32_t)CONFIG_ENCODER_DEVICE_ID, read_val, sizeof(read_val));
    if (read_err != ESP_OK) {
        ESP_LOGW(TAG, "zero read-value TX failed: %s", esp_err_to_name(read_err));
    }

out:
    if (report_out != NULL) {
        *report_out = rep;
    }
    if (rep.sdo_preset_err == ESP_OK) {
        /* Zero applied via CANopen; only the persistence step can have failed. */
        return rep.sdo_save_err;
    }
    if (rep.sdo_preset_err != ESP_ERR_TIMEOUT) {
        return rep.sdo_preset_err;
    }
    /* CAN2.0B fallback path */
    if (rep.mode_pause_err != ESP_OK) {
        return rep.mode_pause_err;
    }
    if (rep.zero_err != ESP_OK) {
        return rep.zero_err;
    }
    if (rep.mode_restore_err != ESP_OK) {
        return rep.mode_restore_err;
    }
    return read_err;
}
