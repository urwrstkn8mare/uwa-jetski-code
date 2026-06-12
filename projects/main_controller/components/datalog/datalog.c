#include "datalog.h"

#include "can.h"
#include "can_ids.h"
#include "config.h"
#include "control.h"
#include "encoder_can.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "height.h"
#include "imu.h"
#include "servo_drive.h"
#include "walltime.h"

#include <dirent.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static const char *TAG = "datalog";

#define MOUNT_POINT  "/data"
#define PART_LABEL   "datalog"
#define MAX_SESSIONS 128
#define HZ_NVS_KEY   "dlog_hz"

/* Free-space thresholds: evict the oldest session below EVICT, warn below WARN. */
#define EVICT_FREE_BYTES (512u * 1024u)
#define WARN_FREE_BYTES  (3u * 512u * 1024u)

/* Per-session meta sidecar (s%08u.tim): sample rate + UTC start epoch. */
typedef struct __attribute__((packed)) {
    uint32_t start_epoch_s;              /* 0 = unknown */
    uint16_t sample_hz;
} session_meta_t;

static SemaphoreHandle_t s_lock;
static FILE     *s_file;                 /* current session, open for append */
static FILE     *s_cfg_file;             /* current config sidecar, open for append */
static uint32_t  s_session_id;
static int64_t   s_session_start_ms;
static uint32_t  s_writes_since_sync;
static bool      s_session_stamped;      /* meta sidecar carries a start epoch */
static uint16_t  s_cfg_hz = DATALOG_DEFAULT_HZ;     /* rate for new sessions */
static uint16_t  s_session_hz = DATALOG_DEFAULT_HZ; /* rate of the active session */

/* Latest GPS from the aux controller (CAN). Plain races are benign — the
 * sampler only ever reads a slightly-stale fix. */
static volatile float s_gps_lat, s_gps_lon, s_gps_speed_kn, s_gps_course;
static volatile bool  s_gps_have;

static void lock(void)   { xSemaphoreTakeRecursive(s_lock, portMAX_DELAY); }
static void unlock(void) { xSemaphoreGiveRecursive(s_lock); }
static void close_session(void);

static int16_t q10(float v) {
    if (v > 3276.7f) return INT16_MAX;
    if (v < -3276.8f) return INT16_MIN;
    return (int16_t)lroundf(v * 10.0f);
}

static void session_path(uint32_t id, char *buf, size_t n) {
    snprintf(buf, n, MOUNT_POINT "/s%08" PRIu32 ".bin", id);
}

static void session_cfg_path(uint32_t id, char *buf, size_t n) {
    snprintf(buf, n, MOUNT_POINT "/s%08" PRIu32 ".cfg", id);
}

static void session_meta_path(uint32_t id, char *buf, size_t n) {
    snprintf(buf, n, MOUNT_POINT "/s%08" PRIu32 ".tim", id);
}

static bool parse_session_id(const char *name, uint32_t *id_out) {
    /* "sXXXXXXXX.bin" */
    if (name[0] != 's') return false;
    char *end = NULL;
    unsigned long id = strtoul(name + 1, &end, 10);
    if (end == NULL || strcmp(end, ".bin") != 0) return false;
    *id_out = (uint32_t)id;
    return true;
}

/* Collect session ids into out[] sorted ascending (oldest first). Returns count. */
static int scan_sessions(uint32_t *out, int max) {
    DIR *d = opendir(MOUNT_POINT);
    if (d == NULL) return 0;
    int n = 0;
    struct dirent *e;
    while ((e = readdir(d)) != NULL && n < max) {
        uint32_t id;
        if (parse_session_id(e->d_name, &id)) out[n++] = id;
    }
    closedir(d);
    for (int i = 1; i < n; i++) {           /* insertion sort, n is small */
        uint32_t v = out[i];
        int j = i - 1;
        while (j >= 0 && out[j] > v) { out[j + 1] = out[j]; j--; }
        out[j + 1] = v;
    }
    return n;
}

static size_t path_size(const char *path) {
    struct stat st;
    if (stat(path, &st) != 0) return 0;
    return (size_t)st.st_size;
}

static size_t file_size(uint32_t id) {
    char path[64];
    session_path(id, path, sizeof(path));
    return path_size(path);
}

static size_t cfg_file_size(uint32_t id) {
    char path[64];
    session_cfg_path(id, path, sizeof(path));
    return path_size(path);
}

static void remove_session_files(uint32_t id) {
    char path[64];
    session_path(id, path, sizeof(path));
    remove(path);
    session_cfg_path(id, path, sizeof(path));
    remove(path);
    session_meta_path(id, path, sizeof(path));
    remove(path);
}

/* Write the current session's meta sidecar. Written at open with the sample
 * rate, and re-written with the UTC start epoch once the wall clock is known.
 * Caller holds the lock. */
static void write_session_meta(void) {
    session_meta_t m = {
        .start_epoch_s = walltime_epoch_at_uptime_ms(s_session_start_ms),
        .sample_hz = s_session_hz,
    };
    char path[64];
    session_meta_path(s_session_id, path, sizeof(path));
    FILE *f = fopen(path, "wb");
    if (f == NULL) return;
    bool ok = fwrite(&m, sizeof(m), 1, f) == 1;
    fclose(f);
    s_session_stamped = ok && m.start_epoch_s != 0;
    if (s_session_stamped) {
        ESP_LOGI(TAG, "session %" PRIu32 " stamped epoch %" PRIu32,
                 s_session_id, m.start_epoch_s);
    }
}

static void try_stamp_session(void) {
    if (s_session_stamped || s_file == NULL) return;
    if (walltime_epoch_at_uptime_ms(s_session_start_ms) == 0) return;
    write_session_meta();
}

static session_meta_t read_session_meta(uint32_t id) {
    session_meta_t m = { .start_epoch_s = 0, .sample_hz = DATALOG_DEFAULT_HZ };
    char path[64];
    session_meta_path(id, path, sizeof(path));
    FILE *f = fopen(path, "rb");
    if (f == NULL) return m;
    if (fread(&m, sizeof(m), 1, f) != 1 || m.sample_hz == 0) {
        m.start_epoch_s = 0;
        m.sample_hz = DATALOG_DEFAULT_HZ;
    }
    fclose(f);
    return m;
}

static size_t free_bytes(void) {
    size_t total = 0, used = 0;
    if (esp_littlefs_info(PART_LABEL, &total, &used) != ESP_OK) return 0;
    return (total > used) ? (total - used) : 0;
}

/* Delete the oldest non-current session while space is critically low. Caller
 * holds the lock. */
static void evict_if_needed(void) {
    while (free_bytes() < EVICT_FREE_BYTES) {
        uint32_t ids[MAX_SESSIONS];
        int n = scan_sessions(ids, MAX_SESSIONS);
        uint32_t victim = 0;
        for (int i = 0; i < n; i++) {
            if (ids[i] != s_session_id) { victim = ids[i]; break; }
        }
        if (victim == 0) break;             /* nothing else to drop */
        remove_session_files(victim);
        ESP_LOGW(TAG, "evicted session %" PRIu32 " (low space)", victim);
    }
}

static uint32_t next_session_id(void) {
    uint32_t ids[MAX_SESSIONS];
    int n = scan_sessions(ids, MAX_SESSIONS);
    return n > 0 ? ids[n - 1] + 1 : 1;
}

void datalog_config_capture(datalog_config_t *out) {
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));

    control_config_t ctrl;
    control_get_cfg(&ctrl);
    out->height_kp = ctrl.height_kp;
    out->height_ki = ctrl.height_ki;
    out->height_kd = ctrl.height_kd;
    out->pitch_kp = ctrl.pitch_kp;
    out->pitch_ki = ctrl.pitch_ki;
    out->pitch_kd = ctrl.pitch_kd;
    out->roll_kp = ctrl.roll_kp;
    out->roll_ki = ctrl.roll_ki;
    out->roll_kd = ctrl.roll_kd;
    out->rudder_exponent_x100 = ctrl.rudder_exponent_x100;
    out->rudder_max_roll_deg = ctrl.rudder_max_roll_deg;
    out->height_enabled = ctrl.height_enabled ? 1u : 0u;
    out->elevon_max_diff_deg = ctrl.elevon_max_diff_deg;
    out->pitch_target_max_deg = ctrl.pitch_target_max_deg;
    out->height_target_cm = ctrl.height_target_cm;

    imu_config_t imu;
    imu_get_cfg(&imu);
    out->imu_pitch_offset_x10 = imu.pitch_offset_deg_x10;
    out->imu_roll_offset_x10 = imu.roll_offset_deg_x10;

    for (int i = 0; i < 2; i++) {
        servo_calibration_t cal;
        servo_drive_get_cal((servo_channel_t)i, &cal);
        out->servo[i][0] = cal.min_pw_us;
        out->servo[i][1] = cal.zero_pw_us;
        out->servo[i][2] = cal.max_pw_us;
        out->servo[i][3] = cal.min_angle_deg;
        out->servo[i][4] = cal.max_angle_deg;
    }
}

void datalog_config_apply(const datalog_config_t *in) {
    if (in == NULL) return;

    control_config_t ctrl = {
        .height_kp = in->height_kp,
        .height_ki = in->height_ki,
        .height_kd = in->height_kd,
        .pitch_kp = in->pitch_kp,
        .pitch_ki = in->pitch_ki,
        .pitch_kd = in->pitch_kd,
        .roll_kp = in->roll_kp,
        .roll_ki = in->roll_ki,
        .roll_kd = in->roll_kd,
        .rudder_exponent_x100 = in->rudder_exponent_x100,
        .rudder_max_roll_deg = in->rudder_max_roll_deg,
        .height_enabled = in->height_enabled != 0,
        .elevon_max_diff_deg = in->elevon_max_diff_deg,
        .pitch_target_max_deg = in->pitch_target_max_deg,
        .height_target_cm = in->height_target_cm,
    };
    control_apply_cfg(&ctrl);

    imu_config_t imu = {
        .pitch_offset_deg_x10 = in->imu_pitch_offset_x10,
        .roll_offset_deg_x10 = in->imu_roll_offset_x10,
    };
    imu_apply_cfg(&imu);

    for (int i = 0; i < 2; i++) {
        servo_calibration_t cal = {
            .min_pw_us = in->servo[i][0],
            .zero_pw_us = in->servo[i][1],
            .max_pw_us = in->servo[i][2],
            .min_angle_deg = in->servo[i][3],
            .max_angle_deg = in->servo[i][4],
        };
        servo_drive_apply_cal((servo_channel_t)i, &cal);
    }
}

static esp_err_t append_config_event_locked(uint32_t t_ms) {
    if (s_cfg_file == NULL) return ESP_ERR_INVALID_STATE;

    datalog_cfgevent_t ev;
    ev.t_ms = t_ms;
    datalog_config_capture(&ev.cfg);
    if (fwrite(&ev, sizeof(ev), 1, s_cfg_file) != 1) return ESP_FAIL;
    fflush(s_cfg_file);
    fsync(fileno(s_cfg_file));
    return ESP_OK;
}

/* Open a fresh session file. Caller holds the lock. */
static esp_err_t open_session(uint32_t id) {
    char path[64];
    session_path(id, path, sizeof(path));
    FILE *f = fopen(path, "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "open session %" PRIu32 " failed", id);
        return ESP_FAIL;
    }

    session_cfg_path(id, path, sizeof(path));
    FILE *cfg = fopen(path, "wb");
    if (cfg == NULL) {
        fclose(f);
        ESP_LOGE(TAG, "open session cfg %" PRIu32 " failed", id);
        return ESP_FAIL;
    }

    s_file = f;
    s_cfg_file = cfg;
    s_session_id = id;
    s_session_start_ms = esp_timer_get_time() / 1000;
    s_writes_since_sync = 0;
    s_session_stamped = false;
    s_session_hz = s_cfg_hz;
    esp_err_t err = append_config_event_locked(0);
    if (err != ESP_OK) {
        close_session();
        return err;
    }
    write_session_meta();
    ESP_LOGI(TAG, "session %" PRIu32 " started at %u Hz", id, s_session_hz);
    return ESP_OK;
}

static void close_session(void) {
    if (s_file != NULL) {
        fflush(s_file);
        fclose(s_file);
        s_file = NULL;
    }
    if (s_cfg_file != NULL) {
        fflush(s_cfg_file);
        fclose(s_cfg_file);
        s_cfg_file = NULL;
    }
}

static void build_record(datalog_record_t *r) {
    memset(r, 0, sizeof(*r));
    r->t_ms = (uint32_t)((esp_timer_get_time() / 1000) - s_session_start_ms);

    float pitch = 0, roll = 0, yaw = 0;
    imu_get_attitude(&pitch, &roll, &yaw);
    r->pitch_x10 = (int16_t)lroundf(pitch * 10.0f);
    r->roll_x10  = (int16_t)lroundf(roll * 10.0f);
    r->yaw_x10   = (int16_t)lroundf(yaw * 10.0f);

    int32_t h = 0;
    height_get_cm(&h);
    r->height_cm = (int16_t)h;

    float rudder = 0;
    encoder_can_get_angle(&rudder);
    r->rudder_x10 = (int16_t)lroundf(rudder * 10.0f);

    control_status_t cs;
    control_get_status(&cs);
    r->pitch_target_x10   = (int16_t)lroundf(cs.pitch_target_deg * 10.0f);
    r->roll_target_x10    = (int16_t)lroundf(cs.roll_target_deg * 10.0f);
    r->joy_pitch_trim_x10 = (int16_t)lroundf(cs.joy_pitch_trim_deg * 10.0f);
    r->elevon_l_x10       = (int16_t)lroundf(cs.elevon_left_deg * 10.0f);
    r->elevon_r_x10       = (int16_t)lroundf(cs.elevon_right_deg * 10.0f);
    r->height_error_x10   = q10(cs.height_pid.error);
    r->height_p_x10       = q10(cs.height_pid.p);
    r->height_i_x10       = q10(cs.height_pid.i);
    r->height_d_x10       = q10(cs.height_pid.d);
    r->height_out_x10     = q10(cs.height_pid.output);
    r->pitch_error_x10    = q10(cs.pitch_pid.error);
    r->pitch_p_x10        = q10(cs.pitch_pid.p);
    r->pitch_i_x10        = q10(cs.pitch_pid.i);
    r->pitch_d_x10        = q10(cs.pitch_pid.d);
    r->pitch_out_x10      = q10(cs.pitch_pid.output);
    r->roll_error_x10     = q10(cs.roll_pid.error);
    r->roll_p_x10         = q10(cs.roll_pid.p);
    r->roll_i_x10         = q10(cs.roll_pid.i);
    r->roll_d_x10         = q10(cs.roll_pid.d);
    r->roll_out_x10       = q10(cs.roll_pid.output);

    control_config_t cfg;
    control_get_cfg(&cfg);
    r->height_target_cm = cfg.height_target_cm;

    if (s_gps_have) {
        r->lat_deg    = s_gps_lat;
        r->lon_deg    = s_gps_lon;
        r->speed_x100 = (int16_t)lroundf(s_gps_speed_kn * 100.0f);
        r->course_x10 = (int16_t)lroundf(s_gps_course * 10.0f);
        r->flags |= DATALOG_FLAG_GPS_VALID;
    }
}

static void sampler_task(void *arg) {
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / s_session_hz));

        datalog_record_t r;
        build_record(&r);

        lock();
        if (s_file != NULL) {
            fwrite(&r, sizeof(r), 1, s_file);
            try_stamp_session();
            if (++s_writes_since_sync >= s_session_hz) {  /* fsync ~once per second */
                /* fflush only drains stdio; littlefs commits the file's size and
                 * blocks on fsync — without it a power cut reverts the file. */
                fflush(s_file);
                fsync(fileno(s_file));
                evict_if_needed();
                s_writes_since_sync = 0;
            }
        }
        unlock();
    }
}

static void gps_rx_cb(const uint8_t buffer[8], uint32_t header_id, uint64_t ts) {
    (void)ts;
    if (buffer == NULL) return;
    if (header_id == CAN_ID_GPS_POSITION) {
        can_gps_position_t p;
        memcpy(&p, buffer, sizeof(p));
        s_gps_lat  = p.lat_deg;
        s_gps_lon  = p.lon_deg;
        s_gps_have = true;
    } else if (header_id == CAN_ID_GPS_VELOCITY) {
        can_gps_velocity_t v;
        memcpy(&v, buffer, sizeof(v));
        s_gps_speed_kn = v.speed_knots;
        s_gps_course   = v.course_deg;
    }
}

esp_err_t datalog_init(void) {
    s_lock = xSemaphoreCreateRecursiveMutex();
    if (s_lock == NULL) return ESP_ERR_NO_MEM;

    esp_vfs_littlefs_conf_t conf = {
        .base_path = MOUNT_POINT,
        .partition_label = PART_LABEL,
        .format_if_mount_failed = true,
        .dont_mount = false,
    };
    esp_err_t err = esp_vfs_littlefs_register(&conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "littlefs mount failed: %s", esp_err_to_name(err));
        return err;
    }

    err = can_register_rx_cb(gps_rx_cb);
    if (err != ESP_OK) ESP_LOGW(TAG, "GPS RX register failed: %s", esp_err_to_name(err));

    uint16_t hz;
    if (config_get_blob(HZ_NVS_KEY, &hz, sizeof(hz)) == ESP_OK &&
        hz >= DATALOG_MIN_HZ && hz <= DATALOG_MAX_HZ) {
        s_cfg_hz = hz;
    }

    lock();
    evict_if_needed();
    err = open_session(next_session_id());
    unlock();
    if (err != ESP_OK) return err;

    if (xTaskCreate(sampler_task, "datalog", 4096, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "sampler task create failed");
        return ESP_ERR_NO_MEM;
    }

    size_t total = 0, used = 0;
    esp_littlefs_info(PART_LABEL, &total, &used);
    ESP_LOGI(TAG, "ready: %u/%u KB used", (unsigned)(used / 1024), (unsigned)(total / 1024));
    return ESP_OK;
}

esp_err_t datalog_new_session(void) {
    lock();
    close_session();
    evict_if_needed();
    esp_err_t err = open_session(next_session_id());
    unlock();
    return err;
}

esp_err_t datalog_log_config_event(void) {
    lock();
    uint32_t t_ms = (uint32_t)((esp_timer_get_time() / 1000) - s_session_start_ms);
    esp_err_t err = append_config_event_locked(t_ms);
    unlock();
    return err;
}

uint32_t datalog_current_session(void) {
    return s_session_id;
}

int datalog_list_sessions(datalog_session_info_t *out, int max) {
    if (out == NULL || max <= 0) return 0;
    uint32_t ids[MAX_SESSIONS];
    lock();
    int n = scan_sessions(ids, MAX_SESSIONS);
    bool low = free_bytes() < WARN_FREE_BYTES;
    /* oldest non-current session is the next eviction victim */
    uint32_t victim = 0;
    for (int i = 0; i < n; i++) {
        if (ids[i] != s_session_id) { victim = ids[i]; break; }
    }
    int w = 0;
    for (int i = n - 1; i >= 0 && w < max; i--) {   /* newest first */
        uint32_t id = ids[i];
        uint32_t count = (uint32_t)(file_size(id) / sizeof(datalog_record_t));
        session_meta_t m = read_session_meta(id);
        out[w].id = id;
        out[w].record_count = count;
        out[w].duration_ms = count > 0 ? (count - 1) * 1000u / m.sample_hz : 0;
        out[w].start_epoch_s = m.start_epoch_s;
        out[w].sample_hz = m.sample_hz;
        out[w].at_risk = low && (id == victim);
        w++;
    }
    unlock();
    return w;
}

esp_err_t datalog_delete_session(uint32_t id) {
    lock();
    esp_err_t err = ESP_OK;
    if (id == s_session_id) {
        err = ESP_ERR_INVALID_STATE;       /* can't delete the active session */
    } else {
        char path[64];
        session_path(id, path, sizeof(path));
        err = (remove(path) == 0) ? ESP_OK : ESP_FAIL;
        session_cfg_path(id, path, sizeof(path));
        remove(path);
        session_meta_path(id, path, sizeof(path));
        remove(path);
    }
    unlock();
    return err;
}

esp_err_t datalog_delete_all(void) {
    lock();
    uint32_t ids[MAX_SESSIONS];
    int n = scan_sessions(ids, MAX_SESSIONS);
    for (int i = 0; i < n; i++) {
        if (ids[i] == s_session_id) continue;
        remove_session_files(ids[i]);
    }
    unlock();
    return ESP_OK;
}

void datalog_storage_info(size_t *total, size_t *used) {
    size_t t = 0, u = 0;
    esp_littlefs_info(PART_LABEL, &t, &u);
    if (total) *total = t;
    if (used) *used = u;
}

uint32_t datalog_session_record_count(uint32_t id) {
    lock();
    uint32_t c = (uint32_t)(file_size(id) / sizeof(datalog_record_t));
    unlock();
    return c;
}

uint32_t datalog_session_cfgevent_count(uint32_t id) {
    lock();
    uint32_t c = (uint32_t)(cfg_file_size(id) / sizeof(datalog_cfgevent_t));
    unlock();
    return c;
}

uint16_t datalog_sample_hz(void) {
    return s_cfg_hz;
}

esp_err_t datalog_set_sample_hz(uint16_t hz) {
    if (hz < DATALOG_MIN_HZ || hz > DATALOG_MAX_HZ) return ESP_ERR_INVALID_ARG;
    if (hz == s_cfg_hz) return ESP_OK;
    s_cfg_hz = hz;
    esp_err_t err = config_set_blob(HZ_NVS_KEY, &hz, sizeof(hz));
    if (err != ESP_OK) ESP_LOGW(TAG, "persist hz failed: %s", esp_err_to_name(err));
    return ESP_OK;
}

void datalog_session_meta(uint32_t id, uint16_t *sample_hz, uint32_t *start_epoch_s) {
    lock();
    session_meta_t m = read_session_meta(id);
    unlock();
    if (sample_hz) *sample_hz = m.sample_hz;
    if (start_epoch_s) *start_epoch_s = m.start_epoch_s;
}

int datalog_read_session(uint32_t id, size_t offset, void *buf, size_t len) {
    char path[64];
    session_path(id, path, sizeof(path));
    lock();
    if (id == s_session_id && s_file != NULL) { fflush(s_file); fsync(fileno(s_file)); }
    FILE *f = fopen(path, "rb");
    if (f == NULL) { unlock(); return -1; }
    int ret = -1;
    if (fseek(f, (long)offset, SEEK_SET) == 0) {
        size_t got = fread(buf, 1, len, f);
        ret = (int)got;
    }
    fclose(f);
    unlock();
    return ret;
}

int datalog_read_session_cfg(uint32_t id, size_t offset, void *buf, size_t len) {
    char path[64];
    session_cfg_path(id, path, sizeof(path));
    lock();
    if (id == s_session_id && s_cfg_file != NULL) { fflush(s_cfg_file); fsync(fileno(s_cfg_file)); }
    FILE *f = fopen(path, "rb");
    if (f == NULL) { unlock(); return -1; }
    int ret = -1;
    if (fseek(f, (long)offset, SEEK_SET) == 0) {
        size_t got = fread(buf, 1, len, f);
        ret = (int)got;
    }
    fclose(f);
    unlock();
    return ret;
}
