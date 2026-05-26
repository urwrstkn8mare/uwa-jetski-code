#include "webui.h"

#include "config.h"
#include "control.h"
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
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "servo_drive.h"

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

static void sse_push_state(void) {
    int64_t now = esp_timer_get_time();
    if (now - s_last_state_push_us < STATE_THROTTLE_US) return;
    s_last_state_push_us = now;

    control_output_t out;
    control_get_last_output(&out);

    char buf[256];
    int n = snprintf(buf, sizeof(buf),
        "{"
        "\"armed\":%s,"
        "\"cal_mode\":%s,"
        "\"target_cm\":%d,"
        "\"elevon_left_deg\":%.1f,"
        "\"elevon_right_deg\":%.1f,"
        "\"height_enabled\":%s"
        "}",
        control_is_armed() ? "true" : "false",
        servo_drive_any_cal_mode() ? "true" : "false",
        control_get_target(),
        (double)out.elevon_left_deg,
        (double)out.elevon_right_deg,
        control_get_height_enabled() ? "true" : "false");
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

static void heartbeat_timer_cb(TimerHandle_t t) {
    (void)t;
    sse_send_comment("heartbeat");
}

static esp_err_t api_get_state(httpd_req_t *req) {
    control_output_t out;
    control_get_last_output(&out);
    char buf[256];
    int n = snprintf(buf, sizeof(buf),
        "{"
        "\"armed\":%s,"
        "\"cal_mode\":%s,"
        "\"target_cm\":%d,"
        "\"elevon_left_deg\":%.1f,"
        "\"elevon_right_deg\":%.1f,"
        "\"height_enabled\":%s"
        "}",
        control_is_armed() ? "true" : "false",
        servo_drive_any_cal_mode() ? "true" : "false",
        control_get_target(),
        (double)out.elevon_left_deg,
        (double)out.elevon_right_deg,
        control_get_height_enabled() ? "true" : "false");
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
    esp_err_t save_err = config_save_servo_cal(handle, &cal);
    if (save_err != ESP_OK) return json_error_resp(req, "NVS save failed");

    /* Recompute effective servo angle range for control limits immediately. */
    servo_calibration_t cal0, cal1;
    servo_drive_get_cal(0, &cal0);
    servo_drive_get_cal(1, &cal1);
    float range0 = fminf(fabsf(cal0.min_angle_deg), cal0.max_angle_deg);
    float range1 = fminf(fabsf(cal1.min_angle_deg), cal1.max_angle_deg);
    control_set_elevon_max_angle(fminf(range0, range1));

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

    if (enabled) {
        control_disarm();
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
    app_config_t cfg;
    config_load(&cfg);

    char buf[1024];
    int n = snprintf(buf, sizeof(buf),
        "{"
        "\"height_kp\":%ld,\"height_ki\":%ld,\"height_kd\":%ld,"
        "\"pitch_kp\":%ld,\"pitch_ki\":%ld,\"pitch_kd\":%ld,"
        "\"roll_kp\":%ld,\"roll_ki\":%ld,\"roll_kd\":%ld,"
        "\"rudder_exponent_x100\":%d,"
        "\"rudder_max_roll_deg\":%d,"
        "\"arm_threshold_pct\":%d,"
        "\"disarm_threshold_pct\":%d,"
        "\"height_enabled\":%s,"
        "\"elevon_max_diff_deg\":%d,"
        "\"servo0_min_pw_us\":%.2f,\"servo0_zero_pw_us\":%.2f,\"servo0_max_pw_us\":%.2f,\"servo0_min_angle_deg\":%.2f,\"servo0_max_angle_deg\":%.2f,"
        "\"servo1_min_pw_us\":%.2f,\"servo1_zero_pw_us\":%.2f,\"servo1_max_pw_us\":%.2f,\"servo1_min_angle_deg\":%.2f,\"servo1_max_angle_deg\":%.2f"
        "}",
        (long)cfg.control.height_kp, (long)cfg.control.height_ki, (long)cfg.control.height_kd,
        (long)cfg.control.pitch_kp, (long)cfg.control.pitch_ki, (long)cfg.control.pitch_kd,
        (long)cfg.control.roll_kp, (long)cfg.control.roll_ki, (long)cfg.control.roll_kd,
        (int)cfg.control.rudder_exponent_x100,
        (int)cfg.control.rudder_max_roll_deg,
        (int)cfg.control.arm_threshold_pct,
        (int)cfg.control.disarm_threshold_pct,
        cfg.control.height_enabled ? "true" : "false",
        (int)cfg.control.elevon_max_diff_deg,
        (double)cfg.servo.channel[0].min_pw_us, (double)cfg.servo.channel[0].zero_pw_us, (double)cfg.servo.channel[0].max_pw_us,
        (double)cfg.servo.channel[0].min_angle_deg, (double)cfg.servo.channel[0].max_angle_deg,
        (double)cfg.servo.channel[1].min_pw_us, (double)cfg.servo.channel[1].zero_pw_us, (double)cfg.servo.channel[1].max_pw_us,
        (double)cfg.servo.channel[1].min_angle_deg, (double)cfg.servo.channel[1].max_angle_deg);

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

    app_config_t cfg;
    config_load(&cfg);

#define PARSE_INT(field, key) \
    if (has_key(buf, key)) { cfg.control.field = (int32_t)parse_number(buf, key); }
#define PARSE_I16(field, key) \
    if (has_key(buf, key)) { cfg.control.field = (int16_t)parse_number(buf, key); }

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
    PARSE_I16(arm_threshold_pct,    "\"arm_threshold_pct\"")
    PARSE_I16(disarm_threshold_pct, "\"disarm_threshold_pct\"")
    PARSE_I16(elevon_max_diff_deg,     "\"elevon_max_diff_deg\"")
    if (has_key(buf, "\"height_enabled\"")) {
        cfg.control.height_enabled = strstr(buf, "\"height_enabled\":true") != NULL;
    }

#undef PARSE_INT
#undef PARSE_I16

    free(buf);

    control_set_cfg(&cfg.control);
    (void)config_save_control_cfg(&cfg.control);

    return json_ok_resp(req);
}

static esp_err_t api_arm(httpd_req_t *req) {
    if (servo_drive_any_cal_mode()) {
        return json_error_resp(req, "calibration active");
    }
    control_arm();
    return json_ok_resp(req);
}

static esp_err_t api_disarm(httpd_req_t *req) {
    control_disarm();
    return json_ok_resp(req);
}

static esp_err_t api_set_target(httpd_req_t *req) {
    size_t buf_sz = req->content_len;
    if (buf_sz > 128) return json_error_resp(req, "payload too large");

    char *buf = malloc(buf_sz + 1);
    if (!buf) return json_error_resp(req, "no mem");

    int ret = httpd_req_recv(req, buf, buf_sz);
    if (ret <= 0) {
        free(buf);
        return json_error_resp(req, "recv failed");
    }
    buf[buf_sz] = 0;

    int16_t h = (int16_t)parse_number(buf, "\"height_cm\"");
    free(buf);

    control_set_target(h);

    return json_ok_resp(req);
}

static esp_err_t index_handler(httpd_req_t *req) {
    size_t len = (size_t)(index_html_end - index_html_start);
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)index_html_start, len);
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
    {.uri = "/api/events",   .method = HTTP_GET,  .handler = sse_handler},
    {.uri = "/api/state",    .method = HTTP_GET,  .handler = api_get_state},
    {.uri = "/api/servos",   .method = HTTP_GET,  .handler = api_get_servos},
    {.uri = "/api/servos",   .method = HTTP_PUT,  .handler = api_put_servo_calibration},
    {.uri = "/api/servos",   .method = HTTP_POST, .handler = api_post_servo_cal_mode},
    {.uri = "/api/servos/raw_pw", .method = HTTP_POST, .handler = api_post_servo_raw_pw},
    {.uri = "/api/config",   .method = HTTP_GET,  .handler = api_get_config},
    {.uri = "/api/config",   .method = HTTP_PUT,  .handler = api_put_config},
    {.uri = "/api/arm",      .method = HTTP_POST, .handler = api_arm},
    {.uri = "/api/disarm",   .method = HTTP_POST, .handler = api_disarm},
    {.uri = "/api/target",   .method = HTTP_POST, .handler = api_set_target},
};

static esp_err_t http_server_init(void) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 16;
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