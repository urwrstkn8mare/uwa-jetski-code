#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/*
 * Briterencoder CAN absolute rotary encoder driver.
 *
 * The fitted encoder is the CANopen variant (device type 0x00010196, CiA 406
 * single-turn): it broadcasts its position cyclically as TPDO1 on CAN ID
 * 0x180 + node id (raw 32-bit LE count, no header bytes) and accepts
 * configuration via SDO on 0x600/0x580 + node id. It ignores the parameter
 * writes from the Briter "CANbus" (CAN2.0B) manual, although it does answer
 * that protocol's 0x01 read-value command. See useful/encoder_protocol_notes.md.
 *
 * Call encoder_can_init() once after can_init() to register the frame
 * decoder; position frames are then decoded automatically by the CAN RX task.
 *
 * Angle (degrees) = signed_count * 360.0 / CONFIG_ENCODER_RESOLUTION
 * where signed_count wraps at ±(resolution/2) so the zero point set on the
 * encoder maps to 0°.
 */

/* Register the encoder frame decoder with the CAN RX layer. Must be called
 * after can_init().
 *
 * @param configure  When true, also transmit the auto-return setup commands to
 *                    the encoder (the controlling node does this; read-only
 *                    consumers such as dashboards pass false). */
esp_err_t encoder_can_init(bool configure);

/*
 * Get the latest encoder angle.
 *
 * @param angle_out  Filled with the current angle in degrees if non-NULL and a
 *                   frame has been received. Positive = clockwise from the
 *                   zeroed position.
 * @return true if at least one frame has been received, false otherwise
 *         (angle_out is left untouched when false).
 */
bool encoder_can_get_angle(float *angle_out);

/* Per-step outcome of a zeroing attempt, for user-facing diagnostics.
 * Steps not attempted hold ESP_ERR_NOT_FINISHED; status bytes are 0xFF and
 * abort codes 0 unless a response was received.
 *
 * Primary path (CANopen variant, the encoder actually fitted): SDO write of
 * preset object 0x6003 = 0, then 0x1010:01 store. Fallback path (CAN2.0B
 * variant): pause auto-return, 0x06 zero, restore auto-return. The fallback
 * only runs when the SDO preset write gets no response at all. */
typedef struct {
    esp_err_t sdo_preset_err;     /* CANopen: preset 0x6003 = 0 */
    uint32_t  sdo_preset_abort;   /* SDO abort code when rejected */
    esp_err_t sdo_save_err;       /* CANopen: store parameters 0x1010:01 */
    uint32_t  sdo_save_abort;
    esp_err_t mode_pause_err;     /* CAN2.0B: pause auto-return (0x04) */
    uint8_t   mode_pause_status;
    esp_err_t zero_err;           /* CAN2.0B: the 0x06 set-zero write */
    uint8_t   zero_status;
    esp_err_t mode_restore_err;   /* CAN2.0B: restore auto-return (0x04) */
    uint8_t   mode_restore_status;
} encoder_zero_report_t;

/* Command the encoder to set its current physical position as zero
 * (persistent across power cycles).
 *
 * @param report_out Optional per-step diagnostics (filled on every return).
 * @return ESP_OK if the encoder acknowledged success, ESP_ERR_TIMEOUT if an
 *         acknowledgement was not received, ESP_FAIL if the encoder
 *         acknowledged an error, or a lower-level TX/setup error. Note: if
 *         the CANopen preset succeeded but the store step failed, the rudder
 *         reads zero now but the zero may not survive a power cycle; the
 *         return value is the store step's error in that case.
 */
esp_err_t encoder_can_zero(encoder_zero_report_t *report_out);
