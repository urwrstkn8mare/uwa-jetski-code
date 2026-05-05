/*
 * Tab5 dashboard: live CAN HUD (with optional demo blend) on the M5 Tab5.
 */

#include "bsp/m5stack_tab5.h"
#include "can.h"
#include "can_ui_bridge.h"
#include "dashboard_demo.h"
#include "dashboard_ui.h"
#include "esp_log.h"
#include "tab5_bringup.h"
#include "ui/dashboard_font.h"

#include <stdint.h>
#include <string.h>

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

typedef struct {
  dashboard_ui_t *ui;
  uint32_t start_ms;
} dashboard_runtime_t;

static dashboard_runtime_t s_runtime;

static void can_rx_cb(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp) {
  can_ui_bridge_on_rx(buffer, header_id, timestamp);
}

static void dashboard_timer_cb(lv_timer_t *timer) {
  static bool logged_first = false;
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
}

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
  s_runtime.ui = dashboard_ui_init(screen, tab5_font_get_cb, NULL);
  if (s_runtime.ui == NULL) {
    ESP_LOGE(TAG, "Failed to create dashboard UI");
    bsp_display_unlock();
    return;
  }

  s_runtime.start_ms = lv_tick_get();

  dashboard_data_t data;
  dashboard_demo_fill(&data, 0);
  dashboard_ui_set_data(s_runtime.ui, &data);

  can_ui_bridge_init();
  if (can_init(can_rx_cb) != ESP_OK) {
    ESP_LOGE(TAG, "CAN init failed");
    bsp_display_unlock();
    return;
  }

  (void)lv_timer_create(dashboard_timer_cb, 50, &s_runtime);
  bsp_display_unlock();
}
