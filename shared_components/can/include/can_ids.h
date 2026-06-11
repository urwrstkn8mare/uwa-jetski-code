#pragma once

#include <stdint.h>

/* Standard IDs (11-bit) — little-endian payloads */

#define CAN_ID_ATTITUDE      0x100u /* can_attitude_t */
#define CAN_ID_HEIGHT        0x101u /* can_height_t */
#define CAN_ID_SERVO_POS     0x103u /* can_servo_pos_t */

#define CAN_ID_GPS_POSITION  0x104u /* can_gps_position_t */
#define CAN_ID_GPS_VELOCITY  0x105u /* can_gps_velocity_t */

#define CAN_ID_CTRL_STATUS   0x106u /* can_ctrl_status_t */
#define CAN_ID_JOYSTICK      0x107u /* can_joystick_t */
#define CAN_ID_RUDDER_ANGLE  0x108u /* can_rudder_angle_t */
#define CAN_ID_CTRL_PERF     0x109u /* can_ctrl_perf_t */

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
    uint8_t  height_target_cm;       /* 0..100 cm, integer */
    int16_t  pitch_target_deg_x10;   /* degrees * 10, e.g. 123 = 12.3° */
    int16_t  roll_target_deg_x10;    /* degrees * 10 */
} can_ctrl_status_t;
_Static_assert(sizeof(can_ctrl_status_t) == 5, "CAN ctrl status frame must be 5 bytes");

/* Joystick Y axis from aux controller (0..100, 50 = centre) */
typedef struct __attribute__((packed)) {
    uint16_t y_pct;  /* up/down: 0=full down, 50=centre, 100=full up */
} can_joystick_t;
_Static_assert(sizeof(can_joystick_t) == 2, "CAN joystick frame must be 2 bytes");

/* Rudder angle re-broadcast by main controller from encoder */
typedef struct __attribute__((packed)) {
    int16_t angle_deg_x10; /* degrees * 10, e.g. 123 = 12.3 degrees */
} can_rudder_angle_t;
_Static_assert(sizeof(can_rudder_angle_t) == 2, "CAN rudder angle frame must be 2 bytes");

/* Control loop performance stats (sent by main controller). */
typedef struct __attribute__((packed)) {
    uint16_t iter_avg_us;  /* avg iteration time in microseconds */
    uint16_t iter_max_us;  /* max iteration time in microseconds */
    uint16_t iter_hz;      /* average iterations per second (Hz) */
} can_ctrl_perf_t;
_Static_assert(sizeof(can_ctrl_perf_t) == 6, "CAN ctrl perf frame must be 6 bytes");
