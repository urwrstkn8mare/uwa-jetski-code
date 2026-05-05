/* Tab5 dashboard: display bring-up + shared dashboard runtime. */

#include "bsp/m5stack_tab5.h"
#include "dashboard_app.h"
#include "esp_log.h"
#include "tab5_bringup.h"
#include "ui/dashboard_font.h"

#include <stdint.h>

static const char *TAG = "main";

extern int FT_Stream_Open(void *stream, const char *filepathname);
static void *__attribute__((used, section(".data"))) force_lv_ftsystem_link_ptr = (void *)FT_Stream_Open;

static const lv_font_t *tab5_font_get_cb(uint16_t size_px, int weight, void *user_data) {
  (void)user_data;
  const lv_font_t *font = NULL;
  if (dashboard_font_get(size_px, weight, &font) == ESP_OK) {
    return font;
  }
  return NULL;
}

static dashboard_app_t s_app;

void app_main(void) {
  lv_display_t *disp = NULL;
  if (tab5_bringup_init(&disp) != ESP_OK) {
    ESP_LOGE(TAG, "Board bring-up failed");
    return;
  }
  (void)disp;

  if (!bsp_display_lock(0)) {
    ESP_LOGE(TAG, "Failed to lock display");
    return;
  }

  lv_obj_t *screen = lv_screen_active();
  lv_display_t *default_disp = lv_display_get_default();
  dashboard_app_config_t cfg = {
      .screen = screen,
      .h_res = default_disp ? lv_display_get_horizontal_resolution(default_disp) : 1280,
      .v_res = default_disp ? lv_display_get_vertical_resolution(default_disp) : 720,
      .can_strip_h = 28,
      .font_get_cb = tab5_font_get_cb,
      .font_user_data = NULL,
      .enable_can_strip = true,
  };
  if (dashboard_app_start(&s_app, &cfg) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start dashboard app");
    bsp_display_unlock();
    return;
  }
  bsp_display_unlock();
}
