#include "dashboard_app.h"

#include "can.h"
#include "can_ui_bridge.h"
#include "dashboard_demo.h"
#include "dashboard_ui.h"
#include "esp_log.h"

static const char *TAG = "dashboard_app";

static void can_rx_cb(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp) {
  can_ui_bridge_on_rx(buffer, header_id, timestamp);
}

static void dashboard_timer_cb(lv_timer_t *timer) {
  static bool logged_first = false;
  static char can_strip_text[192];
  dashboard_app_t *app = lv_timer_get_user_data(timer);
  if (app == NULL || app->ui == NULL) {
    lv_timer_pause(timer);
    return;
  }

  if (!logged_first && can_ui_bridge_got_frame()) {
    logged_first = true;
    ESP_LOGI(TAG, "Receiving CAN data");
  }

  dashboard_data_t data;
  can_ui_bridge_merge_demo(&data, lv_tick_elaps(app->start_ms));
  dashboard_ui_set_data(app->ui, &data);

  if (app->can_strip != NULL) {
    (void)can_ui_bridge_snprintf_status(can_strip_text, sizeof(can_strip_text));
    lv_label_set_text(app->can_strip, can_strip_text);
  }
}

esp_err_t dashboard_app_start(dashboard_app_t *app, const dashboard_app_config_t *cfg) {
  if (app == NULL || cfg == NULL || cfg->screen == NULL || cfg->font_get_cb == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (cfg->h_res <= 0 || cfg->v_res <= 0) {
    return ESP_ERR_INVALID_ARG;
  }

  *app = (dashboard_app_t){0};

  can_ui_bridge_init();
  if (can_init(can_rx_cb) != ESP_OK) {
    ESP_LOGE(TAG, "CAN init failed");
    return ESP_FAIL;
  }

  const int32_t strip_h = (cfg->enable_can_strip && cfg->can_strip_h > 0) ? cfg->can_strip_h : 0;
  const int32_t dashboard_h = cfg->v_res - strip_h;
  if (dashboard_h <= 0) {
    return ESP_ERR_INVALID_SIZE;
  }

  lv_obj_t *dashboard_host = lv_obj_create(cfg->screen);
  lv_obj_remove_style_all(dashboard_host);
  lv_obj_remove_flag(dashboard_host, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_pos(dashboard_host, 0, 0);
  lv_obj_set_size(dashboard_host, cfg->h_res, dashboard_h);
  lv_obj_set_style_bg_opa(dashboard_host, LV_OPA_TRANSP, 0);

  app->ui = dashboard_ui_init(dashboard_host, cfg->font_get_cb, cfg->font_user_data);
  if (app->ui == NULL) {
    ESP_LOGE(TAG, "Failed to create dashboard UI");
    return ESP_FAIL;
  }

  app->start_ms = lv_tick_get();

  dashboard_data_t data;
  dashboard_demo_fill(&data, 0);
  dashboard_ui_set_data(app->ui, &data);

  if (strip_h > 0) {
    lv_obj_t *strip_bg = lv_obj_create(cfg->screen);
    lv_obj_remove_style_all(strip_bg);
    lv_obj_remove_flag(strip_bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(strip_bg, cfg->h_res, strip_h);
    lv_obj_align(strip_bg, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(strip_bg, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(strip_bg, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(strip_bg, 0, 0);

    app->can_strip = lv_label_create(strip_bg);
    lv_obj_set_size(app->can_strip, cfg->h_res - 6, strip_h - 2);
    lv_obj_align(app->can_strip, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(app->can_strip, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(app->can_strip, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(app->can_strip, lv_color_hex(0xCFCFCF), 0);
    lv_label_set_long_mode(app->can_strip, LV_LABEL_LONG_MODE_SCROLL_CIRCULAR);
    lv_label_set_text(app->can_strip, "CAN ... waiting for frames");
  }

  app->timer = lv_timer_create(dashboard_timer_cb, 50, app);
  return ESP_OK;
}
