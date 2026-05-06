/* WS LCD dashboard: CAN/DEMO mode wiring. */

#include "dashboard_can.h"
#include "dashboard_demo.h"
#include "dashboard_ui.h"
#include "esp_log.h"
#include "esp_err.h"
#include "lvgl.h"
#include "lvgl_status_display.h"
#include "sdkconfig.h"
#include "ws_display.h"

#include <stdint.h>
#include <stdio.h>

static const char *TAG = "ws_lcd_main";

static dashboard_ui_t *s_ui;

static const lv_font_t *ws_lcd_font_get_cb(uint16_t size_px, int weight, void *user_data) {
  (void)user_data;
  const lv_font_t *font = NULL;
  esp_err_t err = ws_display_font_get(
      size_px,
      (weight >= 600) ? WS_DISPLAY_FONT_WEIGHT_SEMIBOLD : WS_DISPLAY_FONT_WEIGHT_REGULAR,
      &font);
  return (err == ESP_OK) ? font : NULL;
}

#if !CONFIG_WS_LCD_DASHBOARD_FEED_MODE_DEMO
/* dashboard_can_attach() expects a lock function. */
static esp_err_t ws_can_lock(int32_t timeout_ms, void *ctx) {
  (void)ctx;
  return (ws_display_lock(timeout_ms) == ESP_OK) ? ESP_OK : ESP_FAIL;
}

static void ws_can_unlock(void *ctx) {
  (void)ctx;
  ws_display_unlock();
}

static size_t can_unavailable_status_strip_write(char *buffer, size_t len, void *user) {
  (void)user;
  if (buffer == NULL || len == 0) {
    return 0;
  }

  int n = snprintf(buffer, len, "CAN unavailable");
  return (n > 0) ? (size_t)n : 0;
}
#endif

static size_t demo_status_strip_write(char *buffer, size_t len, void *user) {
  (void)user;
  if (buffer == NULL || len == 0) {
    return 0;
  }

  const int hz = CONFIG_WS_LCD_DASHBOARD_DEMO_HZ;
  int n = snprintf(buffer, len, "DEMO %dHz", (hz > 0) ? hz : 1);
  if (n < 0) {
    return 0;
  }
  return (size_t)n;
}

typedef struct {
  dashboard_ui_t *ui;
  uint32_t start_ms;
} demo_ctx_t;

static demo_ctx_t s_demo_ctx;
static lvgl_status_display_t s_strip;

static void dashboard_demo_update_at(dashboard_ui_t *ui, uint32_t elapsed_ms) {
  if (ui == NULL) {
    return;
  }

  dashboard_data_t data = {0};
  dashboard_demo_fill(&data, elapsed_ms);

  dashboard_ui_set_speed(ui, data.speed_kmh);
  dashboard_ui_set_height(ui, data.height_cm, data.height_target_cm);
  dashboard_ui_set_attitude(ui, data.roll_deg, data.pitch_deg, data.heading_deg);
  dashboard_ui_set_battery(ui, data.battery_percent, data.battery_voltage_v,
                            data.battery_current_a, data.battery_temp_c);

  const int32_t motor_count = (data.motor_count < 0) ? 0 : data.motor_count;
  const int32_t max_motors = (motor_count > DASHBOARD_MOTOR_MAX) ? DASHBOARD_MOTOR_MAX : motor_count;
  for (int32_t i = 0; i < max_motors; i++) {
    dashboard_ui_set_motor(ui, i, data.motor_percent[i], data.motor_power_kw_x10[i],
                             data.motor_rpm[i], data.motor_temp_c[i]);
  }

  dashboard_ui_set_rudder(ui, data.rudder_deg);
  dashboard_ui_set_elevons(ui, data.elevon_left_deg, data.elevon_right_deg);
}

static void demo_timer_cb(lv_timer_t *timer) {
  demo_ctx_t *ctx = lv_timer_get_user_data(timer);
  if (ctx == NULL || ctx->ui == NULL) {
    lv_timer_pause(timer);
    return;
  }

  const uint32_t elapsed_ms = lv_tick_elaps(ctx->start_ms);
  dashboard_demo_update_at(ctx->ui, elapsed_ms);
}

void app_main(void) {
  ESP_ERROR_CHECK(ws_display_init());

  ESP_ERROR_CHECK(ws_display_lock(-1));

  lv_obj_t *screen = lv_screen_active();
  lv_display_t *disp = lv_display_get_default();
  const int32_t h_res = disp ? (int32_t)lv_display_get_horizontal_resolution(disp) : WS_DISPLAY_H_RES;
  const int32_t v_res = disp ? (int32_t)lv_display_get_vertical_resolution(disp) : WS_DISPLAY_V_RES;

  const int32_t strip_h_px = 28;
  const int32_t dashboard_h = v_res - strip_h_px;
  if (dashboard_h <= 0) {
    ESP_LOGE(TAG, "Invalid dashboard area: v_res=%d strip_h=%d", (int)v_res, (int)strip_h_px);
    ws_display_unlock();
    return;
  }

  /* Create a dedicated host container for dashboard_ui. */
  lv_obj_t *dashboard_host = lv_obj_create(screen);
  lv_obj_remove_style_all(dashboard_host);
  lv_obj_remove_flag(dashboard_host, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_pos(dashboard_host, 0, 0);
  lv_obj_set_size(dashboard_host, h_res, dashboard_h);
  lv_obj_set_style_bg_opa(dashboard_host, LV_OPA_TRANSP, 0);

  s_ui = dashboard_ui_init(dashboard_host, ws_lcd_font_get_cb, NULL);
  if (s_ui == NULL) {
    ESP_LOGE(TAG, "Failed to initialise dashboard_ui");
    ws_display_unlock();
    return;
  }

  lv_obj_t *strip_label = dashboard_ui_create_status_strip(screen, h_res, strip_h_px);
  if (strip_label == NULL) {
    ESP_LOGE(TAG, "Failed to create status strip label");
    ws_display_unlock();
    return;
  }

#if CONFIG_WS_LCD_DASHBOARD_FEED_MODE_DEMO
  s_demo_ctx.ui = s_ui;
  s_demo_ctx.start_ms = lv_tick_get();

  const uint32_t demo_hz = (CONFIG_WS_LCD_DASHBOARD_DEMO_HZ > 0) ? (uint32_t)CONFIG_WS_LCD_DASHBOARD_DEMO_HZ : 1u;
  const uint32_t demo_period_ms = (1000u + (demo_hz / 2u)) / demo_hz;

  (void)lv_timer_create(demo_timer_cb, demo_period_ms, &s_demo_ctx);
  dashboard_demo_update_at(s_ui, 0);

  lvgl_status_line_t lines[1] = {
      {.write = demo_status_strip_write, .ctx = NULL},
  };
  ESP_ERROR_CHECK(lvgl_status_display_start(&s_strip, strip_label, lines, 1,
                                              250, 64));
#else
  ESP_ERROR_CHECK(dashboard_can_attach(s_ui, ws_can_lock, ws_can_unlock, NULL));
  lvgl_status_line_t lines[1] = {{0}};
  esp_err_t can_err = dashboard_can_start();
  if (can_err != ESP_OK) {
    ESP_LOGW(TAG, "CAN start failed: %s (UI will stay up)", esp_err_to_name(can_err));
    lines[0].write = can_unavailable_status_strip_write;
    lines[0].ctx = NULL;
  } else {
    lines[0].write = dashboard_can_status_strip_write;
    lines[0].ctx = NULL;
  }

  /* Refresh CAN strip periodically without requiring LVGL redraw knowledge. */
  ESP_ERROR_CHECK(lvgl_status_display_start(&s_strip, strip_label, lines, 1,
                                              250, 96));
#endif

  ws_display_unlock();
}
