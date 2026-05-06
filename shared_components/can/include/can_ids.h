#pragma once

/* Standard IDs (11-bit) — little-endian payloads */

#define CAN_ID_ATTITUDE      0x100u /* int16 pitch_deg, int16 roll_deg, int16 yaw_deg */
#define CAN_ID_HEIGHT        0x101u /* uint16 height_cm */
#define CAN_ID_POTENTIOMETER 0x102u /* uint16 value 0..100 */
#define CAN_ID_SERVO_POS     0x103u /* int16 channel_a_deg, int16 channel_b_deg (e.g. elevons; not rudder) */

#define CAN_ID_GPS_POSITION  0x104u /* float lat_deg, float lon_deg */
#define CAN_ID_GPS_VELOCITY  0x105u /* float speed_knots, float course_deg */
