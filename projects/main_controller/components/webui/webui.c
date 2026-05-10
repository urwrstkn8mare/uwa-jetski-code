#include "webui.h"

#include "config.h"
#include "control.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "webui";

static httpd_handle_t s_server;

/* Embedded file from EMBED_FILES "index.html" */
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

/* ── helpers ── */

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
    while (*p && (*p == ' ' || *p == ':' || *p == '\"' || *p == ',')) p++;
    return strtol(p, NULL, 10);
}

/* ── API handlers ── */

static esp_err_t api_get_state(httpd_req_t *req) {
    control_output_t out;
    control_get_last_output(&out);
    char buf[256];
    int n = snprintf(buf, sizeof(buf),
        "{"
        "\"armed\":%s,"
        "\"target_cm\":%d,"
        "\"elevon_left_deg\":%d,"
        "\"elevon_right_deg\":%d"
        "}",
        control_is_armed() ? "true" : "false",
        control_get_target(),
        out.elevon_left_deg,
        out.elevon_right_deg);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, (size_t)n);
}

static esp_err_t api_get_config(httpd_req_t *req) {
    control_config_t cfg;
    control_get_cfg(&cfg);

    char buf[1024];
    int n = snprintf(buf, sizeof(buf),
        "{"
        "\"height_kp\":%ld,\"height_ki\":%ld,\"height_kd\":%ld,"
        "\"pitch_kp\":%ld,\"pitch_ki\":%ld,\"pitch_kd\":%ld,"
        "\"roll_kp\":%ld,\"roll_ki\":%ld,\"roll_kd\":%ld,"
        "\"rudder_exponent_x100\":%d,"
        "\"rudder_max_roll_deg\":%d,"
        "\"arm_threshold_pct\":%d,"
        "\"disarm_threshold_pct\":%d"
        "}",
        (long)cfg.height_kp, (long)cfg.height_ki, (long)cfg.height_kd,
        (long)cfg.pitch_kp, (long)cfg.pitch_ki, (long)cfg.pitch_kd,
        (long)cfg.roll_kp, (long)cfg.roll_ki, (long)cfg.roll_kd,
        (int)cfg.rudder_exponent_x100,
        (int)cfg.rudder_max_roll_deg,
        (int)cfg.arm_threshold_pct,
        (int)cfg.disarm_threshold_pct);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, (size_t)(n > 0 ? n : 0));
}

static esp_err_t api_put_config(httpd_req_t *req) {
    size_t buf_sz = req->content_len;
    if (buf_sz > 1024) {
        return json_error_resp(req, "payload too large");
    }

    char *buf = malloc(buf_sz + 1);
    if (!buf) {
        return json_error_resp(req, "no mem");
    }

    int ret = httpd_req_recv(req, buf, buf_sz);
    if (ret <= 0) {
        free(buf);
        return json_error_resp(req, "recv failed");
    }
    buf[buf_sz] = 0;

    control_config_t cfg;
    control_get_cfg(&cfg);

#define PARSE(field) cfg.field = (int32_t)parse_number(buf, "\"" #field "\"")
    PARSE(height_kp);
    PARSE(height_ki);
    PARSE(height_kd);
    PARSE(pitch_kp);
    PARSE(pitch_ki);
    PARSE(pitch_kd);
    PARSE(roll_kp);
    PARSE(roll_ki);
    PARSE(roll_kd);
    cfg.rudder_exponent_x100 = (int16_t)parse_number(buf, "\"rudder_exponent_x100\"");
    cfg.rudder_max_roll_deg  = (int16_t)parse_number(buf, "\"rudder_max_roll_deg\"");
    cfg.arm_threshold_pct    = (int16_t)parse_number(buf, "\"arm_threshold_pct\"");
    cfg.disarm_threshold_pct = (int16_t)parse_number(buf, "\"disarm_threshold_pct\"");
#undef PARSE

    free(buf);

    control_set_cfg(&cfg);
    config_save(&cfg);

    return json_ok_resp(req);
}

static esp_err_t api_arm(httpd_req_t *req) {
    control_arm();
    return json_ok_resp(req);
}

static esp_err_t api_disarm(httpd_req_t *req) {
    control_disarm();
    return json_ok_resp(req);
}

static esp_err_t api_set_target(httpd_req_t *req) {
    size_t buf_sz = req->content_len;
    if (buf_sz > 128) {
        return json_error_resp(req, "payload too large");
    }

    char *buf = malloc(buf_sz + 1);
    if (!buf) {
        return json_error_resp(req, "no mem");
    }

    int ret = httpd_req_recv(req, buf, buf_sz);
    if (ret <= 0) {
        free(buf);
        return json_error_resp(req, "recv failed");
    }
    buf[buf_sz] = 0;

    int16_t h = (int16_t)parse_number(buf, "\"height_cm\"");
    control_set_target(h);

    free(buf);
    return json_ok_resp(req);
}

/* ── static file serving ── */

static esp_err_t index_handler(httpd_req_t *req) {
    size_t len = (size_t)(index_html_end - index_html_start);
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)index_html_start, len);
}

/* ── WiFi ── */

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data) {
    (void)arg;
    (void)base;
    (void)data;
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
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                   &wifi_event_handler, NULL),
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

/* ── HTTP server ── */

static const httpd_uri_t s_uris[] = {
    {.uri = "/", .method = HTTP_GET, .handler = index_handler},
    {.uri = "/api/state", .method = HTTP_GET, .handler = api_get_state},
    {.uri = "/api/config", .method = HTTP_GET, .handler = api_get_config},
    {.uri = "/api/config", .method = HTTP_PUT, .handler = api_put_config},
    {.uri = "/api/arm", .method = HTTP_POST, .handler = api_arm},
    {.uri = "/api/disarm", .method = HTTP_POST, .handler = api_disarm},
    {.uri = "/api/target", .method = HTTP_POST, .handler = api_set_target},
};

static esp_err_t http_server_init(void) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 12;
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

/* ── public API ── */

esp_err_t webui_start(void) {
    ESP_RETURN_ON_ERROR(wifi_ap_init(), TAG, "WiFi AP init failed");
    ESP_RETURN_ON_ERROR(http_server_init(), TAG, "HTTP server init failed");
    ESP_LOGI(TAG, "WebUI started — connect to SSID '%s' and open http://192.168.4.1",
             CONFIG_WEBUI_AP_SSID);
    return ESP_OK;
}

void webui_stop(void) {
    if (s_server) {
        httpd_stop(s_server);
        s_server = NULL;
    }
    esp_wifi_stop();
}
