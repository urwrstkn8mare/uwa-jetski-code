#include "datalog.h"

#include "can.h"
#include "can_ids.h"
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

#include <dirent.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

static const char *TAG = "datalog";

#define MOUNT_POINT  "/data"
#define PART_LABEL   "datalog"
#define FLUSH_EVERY  DATALOG_SAMPLE_HZ          /* fsync once per second */
#define MAX_SESSIONS 128

/* Free-space thresholds: evict the oldest session below EVICT, warn below WARN. */
#define EVICT_FREE_BYTES (512u * 1024u)
#define WARN_FREE_BYTES  (3u * 512u * 1024u)

static SemaphoreHandle_t s_lock;
static FILE     *s_file;                 /* current session, open for append */
static uint32_t  s_session_id;
static int64_t   s_session_start_ms;
static uint32_t  s_writes_since_sync;

/* Latest GPS from the aux controller (CAN). Plain races are benign — the
 * sampler only ever reads a slightly-stale fix. */
static volatile float s_gps_lat, s_gps_lon, s_gps_speed_kn, s_gps_course;
static volatile bool  s_gps_have;

static void lock(void)   { xSemaphoreTakeRecursive(s_lock, portMAX_DELAY); }
static void unlock(void) { xSemaphoreGiveRecursive(s_lock); }

static void session_path(uint32_t id, char *buf, size_t n) {
    snprintf(buf, n, MOUNT_POINT "/s%08" PRIu32 ".bin", id);
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

static size_t file_size(uint32_t id) {
    char path[64];
    session_path(id, path, sizeof(path));
    struct stat st;
    if (stat(path, &st) != 0) return 0;
    return (size_t)st.st_size;
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
        char path[64];
        session_path(victim, path, sizeof(path));
        if (remove(path) != 0) break;
        ESP_LOGW(TAG, "evicted session %" PRIu32 " (low space)", victim);
    }
}

static uint32_t next_session_id(void) {
    uint32_t ids[MAX_SESSIONS];
    int n = scan_sessions(ids, MAX_SESSIONS);
    return n > 0 ? ids[n - 1] + 1 : 1;
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
    s_file = f;
    s_session_id = id;
    s_session_start_ms = esp_timer_get_time() / 1000;
    s_writes_since_sync = 0;
    ESP_LOGI(TAG, "session %" PRIu32 " started", id);
    return ESP_OK;
}

static void close_session(void) {
    if (s_file != NULL) {
        fflush(s_file);
        fclose(s_file);
        s_file = NULL;
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
    if (cs.armed) r->flags |= DATALOG_FLAG_ARMED;
}

static void sampler_task(void *arg) {
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(1000 / DATALOG_SAMPLE_HZ);
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, period);

        datalog_record_t r;
        build_record(&r);

        lock();
        if (s_file != NULL) {
            fwrite(&r, sizeof(r), 1, s_file);
            if (++s_writes_since_sync >= FLUSH_EVERY) {
                fflush(s_file);
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
        out[w].id = id;
        out[w].record_count = count;
        out[w].duration_ms = count > 0 ? (count - 1) * (1000u / DATALOG_SAMPLE_HZ) : 0;
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
        char path[64];
        session_path(ids[i], path, sizeof(path));
        remove(path);
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

int datalog_read_session(uint32_t id, size_t offset, void *buf, size_t len) {
    char path[64];
    session_path(id, path, sizeof(path));
    lock();
    if (id == s_session_id && s_file != NULL) fflush(s_file);
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
