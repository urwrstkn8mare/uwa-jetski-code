#pragma once

#include <stdint.h>

/* Standard IDs (11-bit) — little-endian payloads */

#define CAN_ID_ATTITUDE      0x100u /* can_attitude_t */
#define CAN_ID_HEIGHT        0x101u /* can_height_t */
#define CAN_ID_POTENTIOMETER 0x102u /* can_potentiometer_t */
#define CAN_ID_SERVO_POS     0x103u /* can_servo_pos_t */

#define CAN_ID_GPS_POSITION  0x104u /* can_gps_position_t */
#define CAN_ID_GPS_VELOCITY  0x105u /* can_gps_velocity_t */

/* Packed frame structs for compile-time safety.
 * Use these instead of manual memcpy for CAN serialization. */

typedef struct __attribute__((packed)) {
    int16_t pitch_deg;
    int16_t roll_deg;
    int16_t yaw_deg;
} can_attitude_t;
_Static_assert(sizeof(can_attitude_t) == 6, "CAN attitude frame must be 6 bytes");

typedef struct __attribute__((packed)) {
    uint16_t height_cm;
} can_height_t;
_Static_assert(sizeof(can_height_t) == 2, "CAN height frame must be 2 bytes");

typedef struct __attribute__((packed)) {
    uint16_t value;  /* 0..100 */
} can_potentiometer_t;
_Static_assert(sizeof(can_potentiometer_t) == 2, "CAN potentiometer frame must be 2 bytes");

typedef struct __attribute__((packed)) {
    int16_t channel_a_deg;
    int16_t channel_b_deg;
} can_servo_pos_t;
_Static_assert(sizeof(can_servo_pos_t) == 4, "CAN servo pos frame must be 4 bytes");

typedef struct __attribute__((packed)) {
    float lat_deg;
    float lon_deg;
} can_gps_position_t;
_Static_assert(sizeof(can_gps_position_t) == 8, "CAN GPS position frame must be 8 bytes");

typedef struct __attribute__((packed)) {
    float speed_knots;
    float course_deg;
} can_gps_velocity_t;
_Static_assert(sizeof(can_gps_velocity_t) == 8, "CAN GPS velocity frame must be 8 bytes");
