/* WS LCD dashboard: CAN/DEMO mode wiring. */

#include "dashboard_can.h"
#include "dashboard_demo.h"
#include "dashboard_ui.h"
#include "esp_log.h"
#include "esp_err.h"
#include "lvgl.h"
#include "status_ui.h"
#include "sdkconfig.h"
#include "ws_display.h"

#include "freertos/FreeRTOS.h"

#include <stdint.h>
#include <stdio.h>

static const char *TAG = "ws_lcd_main";

static const lv_font_t *ws_lcd_font_get_cb(uint16_t size_px, int weight, void *user_data) {
  (void)user_data;
  const lv_font_t *font = NULL;
  esp_err_t err = ws_display_font_get(
      size_px,
      (weight >= 600) ? WS_DISPLAY_FONT_WEIGHT_SEMIBOLD : WS_DISPLAY_FONT_WEIGHT_REGULAR,
      &font);
  return (err == ESP_OK) ? font : NULL;
}

static void ws_lcd_status_lock(void) { ws_display_lock(portMAX_DELAY); }

static void ws_lcd_status_unlock(void) { ws_display_unlock(); }

#if !CONFIG_WS_LCD_DASHBOARD_FEED_MODE_DEMO
static esp_err_t ws_can_lock(int32_t timeout_ms, void *ctx) {
  (void)ctx;
  return (ws_display_lock(timeout_ms) == ESP_OK) ? ESP_OK : ESP_FAIL;
}

static void ws_can_unlock(void *ctx) {
  (void)ctx;
  ws_display_unlock();
}
#endif

#if CONFIG_WS_LCD_DASHBOARD_FEED_MODE_DEMO
typedef struct {
  uint32_t start_ms;
} demo_ctx_t;

static demo_ctx_t s_demo_ctx;

static void demo_status_timer_cb(lv_timer_t *timer) {
  (void)timer;
  const int hz = (CONFIG_WS_LCD_DASHBOARD_DEMO_HZ > 0) ? CONFIG_WS_LCD_DASHBOARD_DEMO_HZ : 1;
  status_ui_update("Demo", "%dHz", hz);
}

static void demo_timer_cb(lv_timer_t *timer) {
  demo_ctx_t *ctx = lv_timer_get_user_data(timer);
  if (ctx == NULL) {
    lv_timer_pause(timer);
    return;
  }

  const uint32_t elapsed_ms = lv_tick_elaps(ctx->start_ms);
  dashboard_demo_update_ui(elapsed_ms);
}
#endif

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

  lv_obj_t *dashboard_host = lv_obj_create(screen);
  lv_obj_remove_style_all(dashboard_host);
  lv_obj_remove_flag(dashboard_host, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_pos(dashboard_host, 0, 0);
  lv_obj_set_size(dashboard_host, h_res, dashboard_h);
  lv_obj_set_style_bg_opa(dashboard_host, LV_OPA_TRANSP, 0);

  const dashboard_ui_cfg_t ui_cfg = {
      .screen = dashboard_host,
      .font_get_cb = ws_lcd_font_get_cb,
      .font_get_user_data = NULL,
      .lock_cb = ws_lcd_status_lock,
      .unlock_cb = ws_lcd_status_unlock,
  };
  esp_err_t ui_err = dashboard_ui_init(&ui_cfg);
  if (ui_err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialise dashboard_ui: %s", esp_err_to_name(ui_err));
    ws_display_unlock();
    return;
  }

#if CONFIG_WS_LCD_DASHBOARD_FEED_MODE_DEMO
  s_demo_ctx.start_ms = lv_tick_get();

  const uint32_t demo_hz = (CONFIG_WS_LCD_DASHBOARD_DEMO_HZ > 0) ? (uint32_t)CONFIG_WS_LCD_DASHBOARD_DEMO_HZ : 1u;
  const uint32_t demo_period_ms = (1000u + (demo_hz / 2u)) / demo_hz;

  (void)lv_timer_create(demo_timer_cb, demo_period_ms, &s_demo_ctx);
  dashboard_demo_update_ui(0);

  const status_ui_cfg_t cfg = {
      .parent = screen,
      .flex_flow = LV_FLEX_FLOW_ROW,
      .w = h_res,
      .h = strip_h_px,
      .align = LV_ALIGN_BOTTOM_MID,
      .bg_opa = LV_OPA_COVER,
      .lock_cb = ws_lcd_status_lock,
      .unlock_cb = ws_lcd_status_unlock,
      .min_interval_ms = 250,
  };
  ESP_ERROR_CHECK(status_ui_start(&cfg));

  (void)lv_timer_create(demo_status_timer_cb, 250, NULL);
#else
  ESP_ERROR_CHECK(dashboard_can_attach(ws_can_lock, ws_can_unlock, NULL));

  const status_ui_cfg_t cfg = {
      .parent = screen,
      .flex_flow = LV_FLEX_FLOW_ROW,
      .w = h_res,
      .h = strip_h_px,
      .align = LV_ALIGN_BOTTOM_MID,
      .bg_opa = LV_OPA_COVER,
      .lock_cb = ws_lcd_status_lock,
      .unlock_cb = ws_lcd_status_unlock,
      .min_interval_ms = 250,
  };
  ESP_ERROR_CHECK(status_ui_start(&cfg));

  esp_err_t can_err = dashboard_can_start();
  if (can_err != ESP_OK) {
    ESP_LOGW(TAG, "CAN start failed: %s (UI will stay up)", esp_err_to_name(can_err));
    status_ui_update("CAN", "off (TWAI down)");
  }
#endif

  ws_display_unlock();
}
