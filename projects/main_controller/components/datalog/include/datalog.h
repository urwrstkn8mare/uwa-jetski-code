#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Persistent time-series logger for the main controller.
 *
 * A sampler task snapshots all flight channels at DATALOG_SAMPLE_HZ and appends
 * fixed-size records to the current session file on a LittleFS partition
 * ("datalog"). Each power-up / "new session" call starts a fresh session file.
 * When the partition runs low, the oldest sessions are evicted automatically.
 *
 * The on-wire record/header layout below is shared with the webui (the browser
 * parses the same bytes), so it is fixed-point and little-endian.
 */

#define DATALOG_MAGIC     0x444B534Au /* "JSKD" */
#define DATALOG_VERSION   3
#define DATALOG_SAMPLE_HZ 10

/* One logged sample. Fixed-point: *_x10 = value*10, speed_x100 = knots*100.
 * lat/lon are float degrees, 0 when there is no GPS fix (see flags). */
typedef struct __attribute__((packed)) {
    uint32_t t_ms;               /* ms since session start */
    int16_t  pitch_x10;
    int16_t  roll_x10;
    int16_t  yaw_x10;
    int16_t  height_cm;
    int16_t  rudder_x10;
    int16_t  pitch_target_x10;
    int16_t  roll_target_x10;
    int16_t  height_target_cm;
    int16_t  joy_pitch_trim_x10;
    int16_t  elevon_l_x10;
    int16_t  elevon_r_x10;
    int16_t  speed_x100;
    int16_t  course_x10;
    float    lat_deg;
    float    lon_deg;
    uint8_t  flags;              /* bit1: gps_valid */
    uint8_t  _pad;
    int16_t  height_error_x10;
    int16_t  height_p_x10;
    int16_t  height_i_x10;
    int16_t  height_d_x10;
    int16_t  height_out_x10;
    int16_t  pitch_error_x10;
    int16_t  pitch_p_x10;
    int16_t  pitch_i_x10;
    int16_t  pitch_d_x10;
    int16_t  pitch_out_x10;
    int16_t  roll_error_x10;
    int16_t  roll_p_x10;
    int16_t  roll_i_x10;
    int16_t  roll_d_x10;
    int16_t  roll_out_x10;
} datalog_record_t;
_Static_assert(sizeof(datalog_record_t) == 70, "datalog_record_t must be 70 bytes");

#define DATALOG_FLAG_GPS_VALID 0x02u

typedef struct __attribute__((packed)) {
    int32_t height_kp;
    int32_t height_ki;
    int32_t height_kd;
    int32_t pitch_kp;
    int32_t pitch_ki;
    int32_t pitch_kd;
    int32_t roll_kp;
    int32_t roll_ki;
    int32_t roll_kd;
    int16_t rudder_exponent_x100;
    int16_t rudder_max_roll_deg;
    uint8_t height_enabled;
    int16_t elevon_max_diff_deg;
    int16_t pitch_target_max_deg;
    int16_t height_target_cm;
    int16_t imu_pitch_offset_x10;
    int16_t imu_roll_offset_x10;
    float   servo[2][5];
} datalog_config_t;
_Static_assert(sizeof(datalog_config_t) == 91, "datalog_config_t must be 91 bytes");

typedef struct __attribute__((packed)) {
    uint32_t t_ms;
    datalog_config_t cfg;
} datalog_cfgevent_t;
_Static_assert(sizeof(datalog_cfgevent_t) == 95, "datalog_cfgevent_t must be 95 bytes");

/* Container header. In a download / "open file" blob, each session is one
 * [datalog_header_t][record_count * datalog_record_t][cfgevent_count *
 * datalog_cfgevent_t] block; blocks concatenate so a single file can hold many
 * sessions. Counts are filled at serialize time. */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t record_size;
    uint32_t session_id;
    uint32_t record_count;
    uint16_t cfgevent_size;
    uint32_t cfgevent_count;
} datalog_header_t;
_Static_assert(sizeof(datalog_header_t) == 22, "datalog_header_t must be 22 bytes");

typedef struct {
    uint32_t id;
    uint32_t record_count;
    uint32_t duration_ms;
    bool     at_risk;          /* slated for eviction soon (low free space) */
} datalog_session_info_t;

/* Mount the partition, open a fresh session, and start the sampler task. */
esp_err_t datalog_init(void);

/* Close the current session and open a new one. */
esp_err_t datalog_new_session(void);

uint32_t  datalog_current_session(void);

/* Fill out[] with up to max sessions, newest first. Returns the count written. */
int       datalog_list_sessions(datalog_session_info_t *out, int max);

esp_err_t datalog_delete_session(uint32_t id);
esp_err_t datalog_delete_all(void);

/* total/used bytes of the datalog partition (either pointer may be NULL). */
void      datalog_storage_info(size_t *total, size_t *used);

uint32_t  datalog_session_record_count(uint32_t id);
uint32_t  datalog_session_cfgevent_count(uint32_t id);

void      datalog_config_capture(datalog_config_t *out);
void      datalog_config_apply(const datalog_config_t *in);
esp_err_t datalog_log_config_event(void);

/* Random-access read of a session's record bytes (no header). Returns bytes
 * read (0 at EOF) or -1 on error. */
int       datalog_read_session(uint32_t id, size_t offset, void *buf, size_t len);
int       datalog_read_session_cfg(uint32_t id, size_t offset, void *buf, size_t len);
