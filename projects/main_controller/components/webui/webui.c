#include "webui.h"

#include "control.h"
#include "datalog.h"
#include "encoder_can.h"
#include "imu.h"
#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"
#include "servo_drive.h"
#include "walltime.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/fcntl.h>

static const char *TAG = "webui";

static httpd_handle_t s_server;

static int s_sse_fd = -1;

static int64_t s_last_servo_push_us[SERVO_MAX_INSTANCES];
static int64_t s_last_state_push_us;

static TimerHandle_t s_heartbeat_timer;

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");
extern const uint8_t app_css_start[] asm("_binary_app_css_gz_start");
extern const uint8_t app_css_end[]   asm("_binary_app_css_gz_end");
extern const uint8_t app_js_start[] asm("_binary_app_js_gz_start");
extern const uint8_t app_js_end[]   asm("_binary_app_js_gz_end");
extern const uint8_t live_js_start[] asm("_binary_live_js_gz_start");
extern const uint8_t live_js_end[]   asm("_binary_live_js_gz_end");
extern const uint8_t leaflet_js_start[] asm("_binary_leaflet_js_gz_start");
extern const uint8_t leaflet_js_end[]   asm("_binary_leaflet_js_gz_end");
extern const uint8_t leaflet_css_start[] asm("_binary_leaflet_css_gz_start");
extern const uint8_t leaflet_css_end[]   asm("_binary_leaflet_css_gz_end");

static const int64_t SERVO_THROTTLE_US = 100000LL;
static const int64_t STATE_THROTTLE_US  = 200000LL;

static esp_err_t json_error_resp(httpd_req_t *req, const char *msg) {
    char buf[128];
    int n = snprintf(buf, sizeof(buf), "{\"error\":\"%s\"}", msg);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, (size_t)n);
}

static esp_err_t json_ok_resp(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static long parse_number(const char *json, const char *key) {
    const char *p = strstr(json, key);
    if (!p) return 0;
    p += strlen(key);
    while (*p && (*p == ' ' || *p == ':' || *p == '"' || *p == ',')) p++;
    return strtol(p, NULL, 10);
}

static float parse_float(const char *json, const char *key) {
    const char *p = strstr(json, key);
    if (!p) return 0.0f;
    p += strlen(key);
    while (*p && (*p == ' ' || *p == ':' || *p == '"' || *p == ',')) p++;
    return strtof(p, NULL);
}

static bool has_key(const char *json, const char *key) {
    return strstr(json, key) != NULL;
}

static esp_err_t send_raw(int fd, const char *data, size_t len) {
    if (fd < 0 || data == NULL) return ESP_ERR_INVALID_ARG;
    ssize_t sent = send(fd, data, len, 0);
    if (sent < 0) return ESP_FAIL;
    return ESP_OK;
}

static esp_err_t sse_send(const char *json_data) {
    if (s_sse_fd < 0) return ESP_OK;
    char buf[512];
    int n = snprintf(buf, sizeof(buf), "data: %s\n\n", json_data);
    if (n <= 0 || n >= (int)sizeof(buf)) return ESP_ERR_NO_MEM;
    return send_raw(s_sse_fd, buf, (size_t)n);
}

static esp_err_t sse_send_comment(const char *msg) {
    if (s_sse_fd < 0) return ESP_OK;
    char buf[128];
    int n = snprintf(buf, sizeof(buf), ": %s\n\n", msg);
    if (n <= 0) return ESP_FAIL;
    return send_raw(s_sse_fd, buf, (size_t)n);
}

static int build_state_json(char *buf, size_t sz) {
    control_status_t st;
    control_get_status(&st);
    return snprintf(buf, sz,
        "{"
        "\"cal_mode\":%s,"
        "\"elevon_left_deg\":%.1f,"
        "\"elevon_right_deg\":%.1f"
        "}",
        servo_drive_any_cal_mode() ? "true" : "false",
        (double)st.elevon_left_deg,
        (double)st.elevon_right_deg);
}

static void sse_push_state(void) {
    int64_t now = esp_timer_get_time();
    if (now - s_last_state_push_us < STATE_THROTTLE_US) return;
    s_last_state_push_us = now;

    char buf[256];
    int n = build_state_json(buf, sizeof(buf));
    if (n > 0 && n < (int)sizeof(buf)) {
        sse_send(buf);
    }
}

static void sse_push_servo(int idx) {
    if (idx < 0 || idx >= SERVO_MAX_INSTANCES) return;
    int64_t now = esp_timer_get_time();
    if (now - s_last_servo_push_us[idx] < SERVO_THROTTLE_US) return;
    s_last_servo_push_us[idx] = now;

    servo_info_t info;
    if (!servo_drive_get_info_by_index(idx, &info)) return;
    if (!info.in_use) return;

    char buf[256];
    int n = snprintf(buf, sizeof(buf),
        "{"
        "\"type\":\"servo\","
        "\"handle\":%d,"
        "\"gpio\":%d,"
        "\"ready\":%s,"
        "\"simulated\":%s,"
        "\"cal_mode\":%s,"
        "\"cmd_deg\":%.1f"
        "}",
        idx, info.gpio,
        info.ready ? "true" : "false",
        info.simulated ? "true" : "false",
        info.cal_mode ? "true" : "false",
        (double)info.cmd_deg);
    if (n > 0 && n < (int)sizeof(buf)) {
        sse_send(buf);
    }
}

static void sse_notify_servo_change(int idx) {
    sse_push_servo(idx);
    sse_push_state();
}

static void sse_heartbeat_work(void *arg) {
    (void)arg;
    sse_send_comment("heartbeat");
}

/* Runs on the timer service task; defer the socket send to the httpd task. */
static void heartbeat_timer_cb(TimerHandle_t t) {
    (void)t;
    if (s_server) {
        httpd_queue_work(s_server, sse_heartbeat_work, NULL);
    }
}

static esp_err_t api_get_state(httpd_req_t *req) {
    char buf[256];
    int n = build_state_json(buf, sizeof(buf));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, (size_t)(n > 0 ? n : 0));
}

static esp_err_t api_get_servos(httpd_req_t *req) {
    char buf[1024];
    int offset = 0;
    int remaining = (int)sizeof(buf);

    int count = servo_drive_get_count();
    int written = snprintf(buf + offset, (size_t)remaining, "[");
    if (written <= 0 || written >= remaining) {
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, "[]", 2);
    }
    offset += written;
    remaining -= written;

    bool first = true;
    for (int i = 0; i < SERVO_MAX_INSTANCES && count > 0; i++) {
        servo_info_t info;
        if (!servo_drive_get_info_by_index(i, &info)) continue;
        if (!info.in_use) continue;

        written = snprintf(buf + offset, (size_t)remaining,
            "%s{"
            "\"handle\":%d,"
            "\"gpio\":%d,"
            "\"ready\":%s,"
            "\"simulated\":%s,"
            "\"cal_mode\":%s,"
            "\"cmd_deg\":%.1f,"
            "\"cal\":{"
            "\"min_pw_us\":%.2f,"
            "\"zero_pw_us\":%.2f,"
            "\"max_pw_us\":%.2f,"
            "\"min_angle_deg\":%.2f,"
            "\"max_angle_deg\":%.2f}"
            "}",
            first ? "" : ",",
            i, info.gpio,
            info.ready ? "true" : "false",
            info.simulated ? "true" : "false",
            info.cal_mode ? "true" : "false",
            (double)info.cmd_deg,
            (double)info.cal.min_pw_us,
            (double)info.cal.zero_pw_us,
            (double)info.cal.max_pw_us,
            (double)info.cal.min_angle_deg,
            (double)info.cal.max_angle_deg);
        if (written <= 0 || written >= remaining) break;
        offset += written;
        remaining -= written;
        first = false;
        count--;
    }

    if (remaining <= 0) {
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, "[]", 2);
    }

    written = snprintf(buf + offset, (size_t)remaining, "]");
    if (written <= 0) {
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, "[]", 2);
    }
    offset += written;

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, (size_t)offset);
}

static esp_err_t api_put_servo_calibration(httpd_req_t *req) {
    size_t buf_sz = req->content_len;
    if (buf_sz > 512) {
        return json_error_resp(req, "payload too large");
    }

    char *buf = malloc(buf_sz + 1);
    if (!buf) return json_error_resp(req, "no mem");

    int ret = httpd_req_recv(req, buf, buf_sz);
    if (ret <= 0) {
        free(buf);
        return json_error_resp(req, "recv failed");
    }
    buf[buf_sz] = 0;

    int handle = (int)parse_number(buf, "\"handle\"");
    if (handle < 0 || handle >= SERVO_MAX_INSTANCES) {
        free(buf);
        return json_error_resp(req, "invalid handle");
    }

    servo_calibration_t cal;
    memset(&cal, 0, sizeof(cal));
    bool any = false;

    const char *min_pw = strstr(buf, "\"min_pw_us\"");
    if (min_pw) { cal.min_pw_us = parse_float(buf, "\"min_pw_us\""); any = true; }
    const char *zero_pw = strstr(buf, "\"zero_pw_us\"");
    if (zero_pw) { cal.zero_pw_us = parse_float(buf, "\"zero_pw_us\""); any = true; }
    const char *max_pw = strstr(buf, "\"max_pw_us\"");
    if (max_pw) { cal.max_pw_us = parse_float(buf, "\"max_pw_us\""); any = true; }
    const char *min_ang = strstr(buf, "\"min_angle_deg\"");
    if (min_ang) { cal.min_angle_deg = parse_float(buf, "\"min_angle_deg\""); any = true; }
    const char *max_ang = strstr(buf, "\"max_angle_deg\"");
    if (max_ang) { cal.max_angle_deg = parse_float(buf, "\"max_angle_deg\""); any = true; }

    free(buf);

    if (!any) return json_error_resp(req, "no calibration fields");

    if (!(cal.min_pw_us < cal.zero_pw_us && cal.zero_pw_us < cal.max_pw_us))
        return json_error_resp(req, "pw_us must satisfy min < zero < max");
    if (fabsf(cal.min_angle_deg - cal.max_angle_deg) < 1.0f || cal.min_angle_deg * cal.max_angle_deg > 0.0f)
        return json_error_resp(req, "angle range must span zero (one negative, one positive)");

    servo_drive_apply_cal((servo_channel_t)handle, &cal);
    (void)datalog_log_config_event();

    return json_ok_resp(req);
}

static esp_err_t api_post_servo_cal_mode(httpd_req_t *req) {
    size_t buf_sz = req->content_len;
    if (buf_sz > 64) return json_error_resp(req, "payload too large");

    char *buf = malloc(buf_sz + 1);
    if (!buf) return json_error_resp(req, "no mem");

    int ret = httpd_req_recv(req, buf, buf_sz);
    if (ret <= 0) {
        free(buf);
        return json_error_resp(req, "recv failed");
    }
    buf[buf_sz] = 0;

    int handle = (int)parse_number(buf, "\"handle\"");
    bool enabled = strstr(buf, "true") != NULL;
    free(buf);

    if (handle < 0 || handle >= SERVO_MAX_INSTANCES) {
        return json_error_resp(req, "invalid handle");
    }

    servo_drive_set_cal_mode((servo_channel_t)handle, enabled);

    return json_ok_resp(req);
}

static esp_err_t api_post_servo_raw_pw(httpd_req_t *req) {
    size_t buf_sz = req->content_len;
    if (buf_sz > 64) return json_error_resp(req, "payload too large");

    char *buf = malloc(buf_sz + 1);
    if (!buf) return json_error_resp(req, "no mem");

    int ret = httpd_req_recv(req, buf, buf_sz);
    if (ret <= 0) {
        free(buf);
        return json_error_resp(req, "recv failed");
    }
    buf[buf_sz] = 0;

    int handle = (int)parse_number(buf, "\"handle\"");
    float pulse_us = parse_float(buf, "\"pulse_us\"");
    free(buf);

    if (handle < 0 || handle >= SERVO_MAX_INSTANCES) {
        return json_error_resp(req, "invalid handle");
    }

    servo_drive_set_raw_us((servo_channel_t)handle, pulse_us);
    return json_ok_resp(req);
}

static esp_err_t api_get_config(httpd_req_t *req) {
    control_config_t ctrl;
    control_get_cfg(&ctrl);
    servo_calibration_t s0, s1;
    servo_drive_get_cal(0, &s0);
    servo_drive_get_cal(1, &s1);

    char buf[1024];
    int n = snprintf(buf, sizeof(buf),
        "{"
        "\"height_kp\":%ld,\"height_ki\":%ld,\"height_kd\":%ld,"
        "\"pitch_kp\":%ld,\"pitch_ki\":%ld,\"pitch_kd\":%ld,"
        "\"roll_kp\":%ld,\"roll_ki\":%ld,\"roll_kd\":%ld,"
        "\"rudder_exponent_x100\":%d,"
        "\"rudder_max_roll_deg\":%d,"
        "\"height_enabled\":%s,"
        "\"elevon_max_diff_deg\":%d,"
        "\"pitch_target_max_deg\":%d,"
        "\"height_target_cm\":%d,"
        "\"servo0_min_pw_us\":%.2f,\"servo0_zero_pw_us\":%.2f,\"servo0_max_pw_us\":%.2f,\"servo0_min_angle_deg\":%.2f,\"servo0_max_angle_deg\":%.2f,"
        "\"servo1_min_pw_us\":%.2f,\"servo1_zero_pw_us\":%.2f,\"servo1_max_pw_us\":%.2f,\"servo1_min_angle_deg\":%.2f,\"servo1_max_angle_deg\":%.2f"
        "}",
        (long)ctrl.height_kp, (long)ctrl.height_ki, (long)ctrl.height_kd,
        (long)ctrl.pitch_kp, (long)ctrl.pitch_ki, (long)ctrl.pitch_kd,
        (long)ctrl.roll_kp, (long)ctrl.roll_ki, (long)ctrl.roll_kd,
        (int)ctrl.rudder_exponent_x100,
        (int)ctrl.rudder_max_roll_deg,
        ctrl.height_enabled ? "true" : "false",
        (int)ctrl.elevon_max_diff_deg,
        (int)ctrl.pitch_target_max_deg,
        (int)ctrl.height_target_cm,
        (double)s0.min_pw_us, (double)s0.zero_pw_us, (double)s0.max_pw_us,
        (double)s0.min_angle_deg, (double)s0.max_angle_deg,
        (double)s1.min_pw_us, (double)s1.zero_pw_us, (double)s1.max_pw_us,
        (double)s1.min_angle_deg, (double)s1.max_angle_deg);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, (size_t)(n > 0 ? n : 0));
}

static esp_err_t api_get_config_defaults(httpd_req_t *req) {
    control_config_t ctrl;
    control_get_defaults(&ctrl);

    char buf[512];
    int n = snprintf(buf, sizeof(buf),
        "{"
        "\"height_kp\":%ld,\"height_ki\":%ld,\"height_kd\":%ld,"
        "\"pitch_kp\":%ld,\"pitch_ki\":%ld,\"pitch_kd\":%ld,"
        "\"roll_kp\":%ld,\"roll_ki\":%ld,\"roll_kd\":%ld,"
        "\"rudder_exponent_x100\":%d,"
        "\"rudder_max_roll_deg\":%d,"
        "\"height_enabled\":%s,"
        "\"elevon_max_diff_deg\":%d,"
        "\"pitch_target_max_deg\":%d,"
        "\"height_target_cm\":%d"
        "}",
        (long)ctrl.height_kp, (long)ctrl.height_ki, (long)ctrl.height_kd,
        (long)ctrl.pitch_kp, (long)ctrl.pitch_ki, (long)ctrl.pitch_kd,
        (long)ctrl.roll_kp, (long)ctrl.roll_ki, (long)ctrl.roll_kd,
        (int)ctrl.rudder_exponent_x100,
        (int)ctrl.rudder_max_roll_deg,
        ctrl.height_enabled ? "true" : "false",
        (int)ctrl.elevon_max_diff_deg,
        (int)ctrl.pitch_target_max_deg,
        (int)ctrl.height_target_cm);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, (size_t)(n > 0 ? n : 0));
}

static esp_err_t api_put_config(httpd_req_t *req) {
    size_t buf_sz = req->content_len;
    if (buf_sz > 1024) return json_error_resp(req, "payload too large");

    char *buf = malloc(buf_sz + 1);
    if (!buf) return json_error_resp(req, "no mem");

    int ret = httpd_req_recv(req, buf, buf_sz);
    if (ret <= 0) {
        free(buf);
        return json_error_resp(req, "recv failed");
    }
    buf[buf_sz] = 0;

    control_config_t ctrl;
    control_get_cfg(&ctrl);

#define PARSE_INT(field, key) \
    if (has_key(buf, key)) { ctrl.field = (int32_t)parse_number(buf, key); }
#define PARSE_I16(field, key) \
    if (has_key(buf, key)) { ctrl.field = (int16_t)parse_number(buf, key); }

    PARSE_INT(height_kp, "\"height_kp\"")
    PARSE_INT(height_ki, "\"height_ki\"")
    PARSE_INT(height_kd, "\"height_kd\"")
    PARSE_INT(pitch_kp,  "\"pitch_kp\"")
    PARSE_INT(pitch_ki,  "\"pitch_ki\"")
    PARSE_INT(pitch_kd,  "\"pitch_kd\"")
    PARSE_INT(roll_kp,   "\"roll_kp\"")
    PARSE_INT(roll_ki,   "\"roll_ki\"")
    PARSE_INT(roll_kd,   "\"roll_kd\"")
    PARSE_I16(rudder_exponent_x100, "\"rudder_exponent_x100\"")
    PARSE_I16(rudder_max_roll_deg,  "\"rudder_max_roll_deg\"")
    PARSE_I16(elevon_max_diff_deg,  "\"elevon_max_diff_deg\"")
    PARSE_I16(pitch_target_max_deg, "\"pitch_target_max_deg\"")
    PARSE_I16(height_target_cm,     "\"height_target_cm\"")
    if (has_key(buf, "\"height_enabled\"")) {
        ctrl.height_enabled = strstr(buf, "\"height_enabled\":true") != NULL;
    }

#undef PARSE_INT
#undef PARSE_I16

    free(buf);

    control_apply_cfg(&ctrl);
    (void)datalog_log_config_event();

    return json_ok_resp(req);
}

static esp_err_t api_get_imu(httpd_req_t *req) {
    imu_config_t cfg;
    imu_get_cfg(&cfg);
    char buf[128];
    int n = snprintf(buf, sizeof(buf),
        "{\"pitch_offset_deg_x10\":%d,\"roll_offset_deg_x10\":%d}",
        (int)cfg.pitch_offset_deg_x10, (int)cfg.roll_offset_deg_x10);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, (size_t)(n > 0 ? n : 0));
}

static esp_err_t api_get_imu_defaults(httpd_req_t *req) {
    imu_config_t cfg;
    imu_get_defaults(&cfg);
    char buf[128];
    int n = snprintf(buf, sizeof(buf),
        "{\"pitch_offset_deg_x10\":%d,\"roll_offset_deg_x10\":%d}",
        (int)cfg.pitch_offset_deg_x10, (int)cfg.roll_offset_deg_x10);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, (size_t)(n > 0 ? n : 0));
}

static esp_err_t api_put_imu(httpd_req_t *req) {
    size_t buf_sz = req->content_len;
    if (buf_sz > 256) return json_error_resp(req, "payload too large");

    char *buf = malloc(buf_sz + 1);
    if (!buf) return json_error_resp(req, "no mem");

    int ret = httpd_req_recv(req, buf, buf_sz);
    if (ret <= 0) {
        free(buf);
        return json_error_resp(req, "recv failed");
    }
    buf[buf_sz] = 0;

    imu_config_t cfg;
    imu_get_cfg(&cfg);
    if (has_key(buf, "\"pitch_offset_deg_x10\"")) {
        cfg.pitch_offset_deg_x10 = (int16_t)parse_number(buf, "\"pitch_offset_deg_x10\"");
    }
    if (has_key(buf, "\"roll_offset_deg_x10\"")) {
        cfg.roll_offset_deg_x10 = (int16_t)parse_number(buf, "\"roll_offset_deg_x10\"");
    }
    free(buf);

    imu_apply_cfg(&cfg);
    (void)datalog_log_config_event();
    return json_ok_resp(req);
}

static esp_err_t api_get_attitude(httpd_req_t *req) {
    float pitch = 0, roll = 0, yaw = 0;
    imu_get_attitude(&pitch, &roll, &yaw);
    float rudder = 0;
    bool rudder_ready = encoder_can_get_angle(&rudder);
    char buf[160];
    int n = snprintf(buf, sizeof(buf),
        "{\"pitch_deg\":%.2f,\"roll_deg\":%.2f,\"yaw_deg\":%.2f,\"ready\":%s,"
        "\"rudder_deg\":%.2f,\"rudder_ready\":%s}",
        (double)pitch, (double)roll, (double)yaw,
        imu_is_ready() ? "true" : "false",
        (double)rudder, rudder_ready ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, (size_t)(n > 0 ? n : 0));
}

static esp_err_t api_post_rudder_zero(httpd_req_t *req) {
    if (encoder_can_zero() != ESP_OK) {
        return json_error_resp(req, "zero command failed");
    }
    return json_ok_resp(req);
}

static esp_err_t api_post_config_restore(httpd_req_t *req) {
    if (req->content_len != sizeof(datalog_config_t)) {
        return json_error_resp(req, "invalid snapshot size");
    }

    datalog_config_t cfg;
    uint8_t *p = (uint8_t *)&cfg;
    size_t remaining = sizeof(cfg);
    while (remaining > 0) {
        int ret = httpd_req_recv(req, (char *)p, remaining);
        if (ret <= 0) return json_error_resp(req, "recv failed");
        p += ret;
        remaining -= (size_t)ret;
    }

    datalog_config_apply(&cfg);
    (void)datalog_log_config_event();
    return json_ok_resp(req);
}

/* ── Wall clock ── */

static const char *time_source_name(walltime_source_t src) {
    switch (src) {
    case WALLTIME_SOURCE_GPS:     return "gps";
    case WALLTIME_SOURCE_BROWSER: return "browser";
    default:                      return "none";
    }
}

static esp_err_t api_get_time(httpd_req_t *req) {
    uint32_t epoch_s = 0;
    walltime_source_t src = WALLTIME_SOURCE_NONE;
    walltime_get(&epoch_s, &src);
    char buf[96];
    int n = snprintf(buf, sizeof(buf),
        "{\"epoch_s\":%" PRIu32 ",\"source\":\"%s\"}",
        epoch_s, time_source_name(src));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, (size_t)(n > 0 ? n : 0));
}

static esp_err_t api_post_time(httpd_req_t *req) {
    size_t buf_sz = req->content_len;
    if (buf_sz > 64) return json_error_resp(req, "payload too large");

    char buf[65];
    int ret = httpd_req_recv(req, buf, buf_sz);
    if (ret <= 0) return json_error_resp(req, "recv failed");
    buf[ret] = 0;

    long epoch_s = parse_number(buf, "\"epoch_s\"");
    if (epoch_s <= 0) return json_error_resp(req, "invalid epoch_s");

    walltime_set((uint32_t)epoch_s, WALLTIME_SOURCE_BROWSER);
    return json_ok_resp(req);
}

/* ── Data logging ── */

static bool query_u32(httpd_req_t *req, const char *key, uint32_t *out) {
    char query[64], val[16];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return false;
    if (httpd_query_key_value(query, key, val, sizeof(val)) != ESP_OK) return false;
    *out = (uint32_t)strtoul(val, NULL, 10);
    return true;
}

static esp_err_t api_get_sessions(httpd_req_t *req) {
    datalog_session_info_t s[32];
    int n = datalog_list_sessions(s, 32);
    size_t total = 0, used = 0;
    datalog_storage_info(&total, &used);

    const size_t buf_sz = 4096;
    char *buf = malloc(buf_sz);
    if (!buf) return json_error_resp(req, "no mem");
    int off = snprintf(buf, buf_sz,
        "{\"current\":%" PRIu32 ",\"hz\":%d,\"total\":%u,\"used\":%u,\"sessions\":[",
        datalog_current_session(), DATALOG_SAMPLE_HZ, (unsigned)total, (unsigned)used);
    for (int i = 0; i < n && off < (int)buf_sz - 160; i++) {
        off += snprintf(buf + off, buf_sz - off,
            "%s{\"id\":%" PRIu32 ",\"records\":%" PRIu32 ",\"duration_ms\":%" PRIu32
            ",\"start_epoch_s\":%" PRIu32 ",\"at_risk\":%s}",
            i ? "," : "", s[i].id, s[i].record_count, s[i].duration_ms,
            s[i].start_epoch_s, s[i].at_risk ? "true" : "false");
    }
    off += snprintf(buf + off, buf_sz - off, "]}");
    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_send(req, buf, (size_t)off);
    free(buf);
    return err;
}

static esp_err_t api_post_session_new(httpd_req_t *req) {
    return (datalog_new_session() == ESP_OK) ? json_ok_resp(req)
                                             : json_error_resp(req, "new session failed");
}

static esp_err_t api_post_session_delete(httpd_req_t *req) {
    uint32_t id;
    if (!query_u32(req, "id", &id)) return json_error_resp(req, "missing id");
    esp_err_t err = datalog_delete_session(id);
    if (err == ESP_ERR_INVALID_STATE) return json_error_resp(req, "session active");
    return (err == ESP_OK) ? json_ok_resp(req) : json_error_resp(req, "delete failed");
}

static esp_err_t api_post_session_clear(httpd_req_t *req) {
    datalog_delete_all();
    return json_ok_resp(req);
}

/* Stream one session as a [header][records][config events] container block. */
static esp_err_t send_session_block(httpd_req_t *req, uint32_t id, char *buf, size_t buf_sz) {
    uint32_t count = datalog_session_record_count(id);
    uint32_t cfg_count = datalog_session_cfgevent_count(id);
    datalog_header_t hdr = {
        .magic = DATALOG_MAGIC,
        .version = DATALOG_VERSION,
        .record_size = sizeof(datalog_record_t),
        .session_id = id,
        .record_count = count,
        .cfgevent_size = sizeof(datalog_cfgevent_t),
        .cfgevent_count = cfg_count,
        .sample_hz = DATALOG_SAMPLE_HZ,
        .start_epoch_s = datalog_session_start_epoch_s(id),
    };
    if (httpd_resp_send_chunk(req, (const char *)&hdr, sizeof(hdr)) != ESP_OK) return ESP_FAIL;

    size_t total = (size_t)count * sizeof(datalog_record_t);
    size_t sent = 0;
    while (sent < total) {
        size_t want = total - sent;
        if (want > buf_sz) want = buf_sz;
        int got = datalog_read_session(id, sent, buf, want);
        if (got <= 0) break;
        if (httpd_resp_send_chunk(req, buf, (size_t)got) != ESP_OK) return ESP_FAIL;
        sent += (size_t)got;
    }

    total = (size_t)cfg_count * sizeof(datalog_cfgevent_t);
    sent = 0;
    while (sent < total) {
        size_t want = total - sent;
        if (want > buf_sz) want = buf_sz;
        int got = datalog_read_session_cfg(id, sent, buf, want);
        if (got <= 0) break;
        if (httpd_resp_send_chunk(req, buf, (size_t)got) != ESP_OK) return ESP_FAIL;
        sent += (size_t)got;
    }
    return ESP_OK;
}

static esp_err_t api_get_session(httpd_req_t *req) {
    uint32_t id;
    if (!query_u32(req, "id", &id)) return json_error_resp(req, "missing id");

    char *buf = malloc(1024);
    if (!buf) return json_error_resp(req, "no mem");

    httpd_resp_set_type(req, "application/octet-stream");
    esp_err_t err = send_session_block(req, id, buf, 1024);
    free(buf);
    if (err != ESP_OK) return err;
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t api_get_download(httpd_req_t *req) {
    datalog_session_info_t s[32];
    int n = datalog_list_sessions(s, 32);

    char *buf = malloc(1024);
    if (!buf) return json_error_resp(req, "no mem");

    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"jetski_log.bin\"");
    for (int i = 0; i < n; i++) {
        if (send_session_block(req, s[i].id, buf, 1024) != ESP_OK) {
            free(buf);
            return ESP_FAIL;
        }
    }
    free(buf);
    return httpd_resp_send_chunk(req, NULL, 0);
}

/* Static assets carry an ETag from the firmware build, so browsers revalidate
 * on each load: unchanged files cost a 304, and every reflash invalidates. */
static esp_err_t send_asset(httpd_req_t *req, const char *type, bool gzip,
                            const uint8_t *start, const uint8_t *end) {
    const esp_app_desc_t *app = esp_app_get_description();
    char etag[16];
    snprintf(etag, sizeof(etag), "\"%02x%02x%02x%02x\"",
             app->app_elf_sha256[0], app->app_elf_sha256[1],
             app->app_elf_sha256[2], app->app_elf_sha256[3]);
    httpd_resp_set_hdr(req, "ETag", etag);
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

    char inm[16];
    if (httpd_req_get_hdr_value_str(req, "If-None-Match", inm, sizeof(inm)) == ESP_OK &&
        strcmp(inm, etag) == 0) {
        httpd_resp_set_status(req, "304 Not Modified");
        return httpd_resp_send(req, NULL, 0);
    }

    httpd_resp_set_type(req, type);
    if (gzip) httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    return httpd_resp_send(req, (const char *)start, (size_t)(end - start));
}

static esp_err_t index_handler(httpd_req_t *req) {
    return send_asset(req, "text/html", false, index_html_start, index_html_end);
}

static esp_err_t app_css_handler(httpd_req_t *req) {
    return send_asset(req, "text/css", true, app_css_start, app_css_end);
}

static esp_err_t app_js_handler(httpd_req_t *req) {
    return send_asset(req, "application/javascript", true, app_js_start, app_js_end);
}

static esp_err_t live_js_handler(httpd_req_t *req) {
    return send_asset(req, "application/javascript", true, live_js_start, live_js_end);
}

static esp_err_t leaflet_js_handler(httpd_req_t *req) {
    return send_asset(req, "application/javascript", true, leaflet_js_start, leaflet_js_end);
}

static esp_err_t leaflet_css_handler(httpd_req_t *req) {
    return send_asset(req, "text/css", true, leaflet_css_start, leaflet_css_end);
}

static esp_err_t sse_handler(httpd_req_t *req) {
    if (s_sse_fd >= 0) {
        close(s_sse_fd);
        s_sse_fd = -1;
    }

    int client_fd = httpd_req_to_sockfd(req);

    static const char hdr[] =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/event-stream\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: keep-alive\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "\r\n";
    ssize_t sent = send(client_fd, hdr, strlen(hdr), 0);
    if (sent < 0) {
        close(client_fd);
        return ESP_FAIL;
    }

    s_sse_fd = client_fd;

    sse_send_comment("connected");
    sse_push_state();

    return ESP_OK;
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data) {
    (void)arg; (void)base; (void)data;
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "WiFi client connected");
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        ESP_LOGI(TAG, "WiFi client disconnected");
    }
}

static esp_err_t wifi_ap_init(void) {
    esp_netif_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&wcfg), TAG, "wifi_init failed");

    ESP_RETURN_ON_ERROR(
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL),
        TAG, "event handler register failed");

    wifi_config_t ap_cfg = {
        .ap = {
            .ssid = CONFIG_WEBUI_AP_SSID,
            .ssid_len = 0,
            .channel = CONFIG_WEBUI_AP_CHANNEL,
            .max_connection = CONFIG_WEBUI_AP_MAX_CONN,
            .authmode = WIFI_AUTH_OPEN,
        },
    };

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "wifi_set_mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg), TAG, "wifi_set_config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi_start failed");

    ESP_LOGI(TAG, "WiFi AP started: SSID='%s' channel=%d",
             CONFIG_WEBUI_AP_SSID, CONFIG_WEBUI_AP_CHANNEL);
    return ESP_OK;
}

static const httpd_uri_t s_uris[] = {
    {.uri = "/",             .method = HTTP_GET,  .handler = index_handler},
    {.uri = "/app.css",      .method = HTTP_GET,  .handler = app_css_handler},
    {.uri = "/app.js",       .method = HTTP_GET,  .handler = app_js_handler},
    {.uri = "/live.js",      .method = HTTP_GET,  .handler = live_js_handler},
    {.uri = "/leaflet.js",   .method = HTTP_GET,  .handler = leaflet_js_handler},
    {.uri = "/leaflet.css",  .method = HTTP_GET,  .handler = leaflet_css_handler},
    {.uri = "/api/events",   .method = HTTP_GET,  .handler = sse_handler},
    {.uri = "/api/state",    .method = HTTP_GET,  .handler = api_get_state},
    {.uri = "/api/servos",   .method = HTTP_GET,  .handler = api_get_servos},
    {.uri = "/api/servos",   .method = HTTP_PUT,  .handler = api_put_servo_calibration},
    {.uri = "/api/servos",   .method = HTTP_POST, .handler = api_post_servo_cal_mode},
    {.uri = "/api/servos/raw_pw", .method = HTTP_POST, .handler = api_post_servo_raw_pw},
    {.uri = "/api/config",   .method = HTTP_GET,  .handler = api_get_config},
    {.uri = "/api/config",   .method = HTTP_PUT,  .handler = api_put_config},
    {.uri = "/api/config/restore", .method = HTTP_POST, .handler = api_post_config_restore},
    {.uri = "/api/config/defaults", .method = HTTP_GET, .handler = api_get_config_defaults},
    {.uri = "/api/imu",      .method = HTTP_GET,  .handler = api_get_imu},
    {.uri = "/api/imu",      .method = HTTP_PUT,  .handler = api_put_imu},
    {.uri = "/api/imu/defaults", .method = HTTP_GET, .handler = api_get_imu_defaults},
    {.uri = "/api/attitude", .method = HTTP_GET,  .handler = api_get_attitude},
    {.uri = "/api/rudder/zero", .method = HTTP_POST, .handler = api_post_rudder_zero},
    {.uri = "/api/time",     .method = HTTP_GET,  .handler = api_get_time},
    {.uri = "/api/time",     .method = HTTP_POST, .handler = api_post_time},
    {.uri = "/api/sessions",        .method = HTTP_GET,  .handler = api_get_sessions},
    {.uri = "/api/session/new",     .method = HTTP_POST, .handler = api_post_session_new},
    {.uri = "/api/session/delete",  .method = HTTP_POST, .handler = api_post_session_delete},
    {.uri = "/api/session/clear",   .method = HTTP_POST, .handler = api_post_session_clear},
    {.uri = "/api/session",         .method = HTTP_GET,  .handler = api_get_session},
    {.uri = "/api/download",        .method = HTTP_GET,  .handler = api_get_download},
};

static esp_err_t http_server_init(void) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    /* Default 4 KB overflows: session handlers stack ~2 KB of buffers on top
     * of LittleFS traversal and float printf. */
    cfg.stack_size = 8192;
    cfg.max_uri_handlers = 30;
    cfg.lru_purge_enable = true;
    cfg.server_port = CONFIG_WEBUI_HTTP_PORT;

    ESP_RETURN_ON_ERROR(httpd_start(&s_server, &cfg), TAG, "httpd_start failed");

    for (size_t i = 0; i < sizeof(s_uris) / sizeof(s_uris[0]); i++) {
        esp_err_t e = httpd_register_uri_handler(s_server, &s_uris[i]);
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "Failed to register %s: %s", s_uris[i].uri, esp_err_to_name(e));
        }
    }

    ESP_LOGI(TAG, "HTTP server running on port %d", cfg.server_port);
    return ESP_OK;
}

esp_err_t webui_start(void) {
    memset(s_last_servo_push_us, 0, sizeof(s_last_servo_push_us));
    s_last_state_push_us = 0;

    control_register_change_cb(sse_push_state);
    servo_drive_register_change_cb(sse_notify_servo_change);

    ESP_RETURN_ON_ERROR(wifi_ap_init(), TAG, "WiFi AP init failed");
    ESP_RETURN_ON_ERROR(http_server_init(), TAG, "HTTP server init failed");

    s_heartbeat_timer = xTimerCreate("sse_hb", pdMS_TO_TICKS(3000),
                                      true, NULL, heartbeat_timer_cb);
    if (s_heartbeat_timer) {
        xTimerStart(s_heartbeat_timer, 0);
    }

    ESP_LOGI(TAG, "WebUI started — connect to SSID '%s' and open http://192.168.4.1",
             CONFIG_WEBUI_AP_SSID);
    return ESP_OK;
}

void webui_stop(void) {
    if (s_heartbeat_timer) {
        xTimerStop(s_heartbeat_timer, 0);
        xTimerDelete(s_heartbeat_timer, 0);
        s_heartbeat_timer = NULL;
    }
    if (s_sse_fd >= 0) {
        close(s_sse_fd);
        s_sse_fd = -1;
    }
    if (s_server) {
        httpd_stop(s_server);
        s_server = NULL;
    }
    esp_wifi_stop();
}
