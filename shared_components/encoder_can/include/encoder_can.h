#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/*
 * Briterencoder CAN absolute rotary encoder driver.
 *
 * The encoder operates in auto-return mode: it periodically sends its current
 * position as a CAN frame.  Call encoder_can_init() once after can_init() to
 * register the frame decoder; encoder frames are then decoded automatically by
 * the CAN RX task.
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
