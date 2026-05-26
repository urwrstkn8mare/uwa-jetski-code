#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/*
 * Briterencoder CAN absolute rotary encoder driver.
 *
 * The encoder operates in auto-return mode: it periodically sends its current
 * position as a CAN frame.  Call encoder_can_init() once after CAN is up to
 * configure the encoder, then route every incoming CAN frame through
 * encoder_can_on_rx().
 *
 * Frame format (encoder → host, CAN ID = encoder device address):
 *   data[0] = 0x07  (frame length)
 *   data[1] = device address
 *   data[2] = 0x01  (read-value command)
 *   data[3..6] = 32-bit LE raw encoder count
 *
 * Angle (degrees) = signed_count * 360.0 / CONFIG_ENCODER_RESOLUTION
 * where signed_count wraps at ±(resolution/2) so the zero point set on the
 * encoder maps to 0°.
 */

/* Configure encoder to auto-return mode and store runtime parameters.
 * Must be called after can_init(). */
esp_err_t encoder_can_init(void);

/* Feed a raw CAN frame into the encoder driver.  Call from on_can_rx(). */
void encoder_can_on_rx(const uint8_t buffer[8], uint32_t header_id);

/*
 * Return true if a fresh encoder reading is available.
 *
 * @param max_age_ms  Maximum acceptable age of the last frame in ms.
 * @param angle_out   Filled with the current angle in degrees if non-NULL.
 *                    Positive = clockwise from the zeroed position.
 */
bool encoder_can_is_fresh(uint32_t max_age_ms, float *angle_out);
