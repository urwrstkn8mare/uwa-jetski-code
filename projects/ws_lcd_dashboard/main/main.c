/* WS LCD dashboard: display bring-up + shared dashboard runtime. */

#include "dashboard_app.h"
#include "esp_log.h"
#include "ws_display.h"

#include <stdint.h>

static const char *TAG = "main";
static dashboard_app_t s_app;

static const lv_font_t *ws_lcd_font_get_cb(uint16_t size_px, int weight, void *user_data) {
  (void)user_data;
  const lv_font_t *font = NULL;
  esp_err_t err = ws_display_font_get(size_px,
                                      (weight >= 600) ? WS_DISPLAY_FONT_WEIGHT_SEMIBOLD : WS_DISPLAY_FONT_WEIGHT_REGULAR,
                                      &font);
  return (err == ESP_OK) ? font : NULL;
}

void app_main(void) {
  ESP_ERROR_CHECK(ws_display_init());
  ESP_ERROR_CHECK(ws_display_lock(-1));

  lv_obj_t *screen = lv_screen_active();
  lv_display_t *disp = lv_display_get_default();
  dashboard_app_config_t cfg = {
      .screen = screen,
      .h_res = disp ? lv_display_get_horizontal_resolution(disp) : 360,
      .v_res = disp ? lv_display_get_vertical_resolution(disp) : 360,
      .can_strip_h = 28,
      .font_get_cb = ws_lcd_font_get_cb,
      .font_user_data = NULL,
      .enable_can_strip = true,
  };
  if (dashboard_app_start(&s_app, &cfg) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start dashboard app");
    ws_display_unlock();
    return;
  }

  ws_display_unlock();
}
