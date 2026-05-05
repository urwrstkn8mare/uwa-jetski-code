/*
 * WS ring LCD dashboard: live CAN HUD (with optional demo blend) on the Espressif round display kit.
 */

#include "can.h"
#include "can_ui_bridge.h"
#include "dashboard_demo.h"
#include "dashboard_ui.h"
#include "esp_err.h"
#include "esp_log.h"
#include "ws_display.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdint.h>

static const char *TAG = "main";

typedef struct {
  dashboard_ui_t *ui;
  uint32_t start_ms;
  lv_obj_t *can_strip;
} dashboard_runtime_t;

static dashboard_runtime_t s_runtime;

static const lv_font_t *ws_lcd_font_get_cb(uint16_t size_px, int weight, void *user_data) {
  (void)user_data;
  const lv_font_t *font = NULL;
  esp_err_t err = ws_display_font_get(size_px,
                                      (weight >= 600) ? WS_DISPLAY_FONT_WEIGHT_SEMIBOLD : WS_DISPLAY_FONT_WEIGHT_REGULAR,
                                      &font);
  return (err == ESP_OK) ? font : NULL;
}

static void can_rx_cb(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp) {
  can_ui_bridge_on_rx(buffer, header_id, timestamp);
}

static void dashboard_timer_cb(lv_timer_t *timer) {
  static bool logged_first = false;
  static char can_strip_text[192];
  dashboard_runtime_t *runtime = lv_timer_get_user_data(timer);
  if (runtime == NULL || runtime->ui == NULL) {
    lv_timer_pause(timer);
    return;
  }

  if (!logged_first && can_ui_bridge_got_frame()) {
    logged_first = true;
    ESP_LOGI(TAG, "Receiving CAN data");
  }

  dashboard_data_t data;
  can_ui_bridge_merge_demo(&data, lv_tick_elaps(runtime->start_ms));
  dashboard_ui_set_data(runtime->ui, &data);

  if (runtime->can_strip != NULL) {
    char can_line[128];
    can_ui_bridge_debug_t dbg = {0};
    can_ui_bridge_get_debug(&dbg);
    (void)can_snprintf_metrics_line(can_line, sizeof(can_line));
    snprintf(can_strip_text, sizeof(can_strip_text),
             "%s | H:%s%" PRId16 "cm Rud:%s%u%% Elv:%s%" PRId16 "/%" PRId16,
             can_line,
             dbg.have_height ? "" : "--", dbg.have_height ? dbg.height_cm : 0,
             dbg.have_pot ? "" : "--", dbg.have_pot ? (unsigned)dbg.pot_pct : 0u,
             dbg.have_servo ? "" : "--", dbg.have_servo ? dbg.servo_a_deg : 0,
             dbg.have_servo ? dbg.servo_b_deg : 0);
    lv_label_set_text(runtime->can_strip, can_strip_text);
  }
}

void app_main(void) {
  can_ui_bridge_init();
  if (can_init(can_rx_cb) != ESP_OK) {
    ESP_LOGE(TAG, "CAN init failed");
    return;
  }

  ESP_ERROR_CHECK(ws_display_init());
  ESP_ERROR_CHECK(ws_display_lock(-1));

  lv_obj_t *screen = lv_screen_active();
  lv_display_t *disp = lv_display_get_default();
  int32_t h_res = disp ? lv_display_get_horizontal_resolution(disp) : 360;
  int32_t v_res = disp ? lv_display_get_vertical_resolution(disp) : 360;
  const int32_t strip_h = 28;

  lv_obj_t *dashboard_host = lv_obj_create(screen);
  lv_obj_remove_style_all(dashboard_host);
  lv_obj_remove_flag(dashboard_host, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_pos(dashboard_host, 0, 0);
  lv_obj_set_size(dashboard_host, h_res, v_res - strip_h);
  lv_obj_set_style_bg_opa(dashboard_host, LV_OPA_TRANSP, 0);

  s_runtime.ui = dashboard_ui_init(dashboard_host, ws_lcd_font_get_cb, NULL);
  if (s_runtime.ui == NULL) {
    ESP_LOGE(TAG, "Failed to create dashboard UI");
    ws_display_unlock();
    return;
  }

  s_runtime.start_ms = lv_tick_get();

  dashboard_data_t data;
  dashboard_demo_fill(&data, 0);
  dashboard_ui_set_data(s_runtime.ui, &data);

  lv_obj_t *strip_bg = lv_obj_create(screen);
  lv_obj_remove_style_all(strip_bg);
  lv_obj_remove_flag(strip_bg, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(strip_bg, h_res, strip_h);
  lv_obj_align(strip_bg, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_set_style_bg_color(strip_bg, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(strip_bg, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(strip_bg, 0, 0);

  s_runtime.can_strip = lv_label_create(strip_bg);
  lv_obj_set_size(s_runtime.can_strip, h_res - 6, strip_h - 2);
  lv_obj_align(s_runtime.can_strip, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(s_runtime.can_strip, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(s_runtime.can_strip, LV_OPA_COVER, 0);
  lv_obj_set_style_text_color(s_runtime.can_strip, lv_color_hex(0xCFCFCF), 0);
  lv_label_set_long_mode(s_runtime.can_strip, LV_LABEL_LONG_MODE_SCROLL_CIRCULAR);
  lv_label_set_text(s_runtime.can_strip, "CAN ... waiting for frames");

  (void)lv_timer_create(dashboard_timer_cb, 50, &s_runtime);

  ws_display_unlock();
}
