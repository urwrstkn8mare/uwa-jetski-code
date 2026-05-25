#pragma once

#include <stdint.h>

/* Standard IDs (11-bit) — little-endian payloads */

#define CAN_ID_ATTITUDE      0x100u /* can_attitude_t */
#define CAN_ID_HEIGHT        0x101u /* can_height_t */
#define CAN_ID_POTENTIOMETER 0x102u /* can_potentiometer_t */
#define CAN_ID_SERVO_POS     0x103u /* can_servo_pos_t */

#define CAN_ID_GPS_POSITION  0x104u /* can_gps_position_t */
#define CAN_ID_GPS_VELOCITY  0x105u /* can_gps_velocity_t */

#define CAN_ID_CTRL_STATUS   0x106u /* can_ctrl_status_t */

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
    uint8_t channel;  /* 0 = A (left), 1 = B (right) */
    float deg;
} can_servo_pos_t;
_Static_assert(sizeof(can_servo_pos_t) == 5, "CAN servo pos frame must be 5 bytes");

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

typedef struct __attribute__((packed)) {
    int16_t height_target_cm;
    int16_t height_current_cm;
    int8_t  pitch_deg;
    int8_t  roll_deg;
    uint8_t flags;      /* bit 0: armed */
} can_ctrl_status_t;
_Static_assert(sizeof(can_ctrl_status_t) == 7, "CAN ctrl status frame must be 7 bytes");
