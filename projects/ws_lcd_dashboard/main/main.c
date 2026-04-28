#include "esp_err.h"
#include "esp_log.h"
#include "dashboard_demo.h"
#include "dashboard_ui.h"
#include "ws_display.h"

#include <stdint.h>

static const char *TAG = "main";

typedef struct {
  dashboard_ui_t *ui;
  uint32_t start_ms;
  uint32_t fps_window_start_ms;
  uint32_t fps_frame_count;
  uint32_t fps_slowest_frame_ms;
} dashboard_runtime_t;

static dashboard_runtime_t s_runtime;

static const lv_font_t *ws_lcd_font_get_cb(uint16_t size_px, int weight, void *user_data)
{
  (void)user_data;
  const lv_font_t *font = NULL;
  esp_err_t err = ws_display_font_get(size_px,
      (weight >= 600)
      ? WS_DISPLAY_FONT_WEIGHT_SEMIBOLD : WS_DISPLAY_FONT_WEIGHT_REGULAR,
      &font);
  return (err == ESP_OK) ? font : NULL;
}

static void dashboard_timer_cb(lv_timer_t *timer) {
  dashboard_runtime_t *runtime = lv_timer_get_user_data(timer);
  if (runtime == NULL || runtime->ui == NULL) {
    lv_timer_pause(timer);
    return;
  }

  const uint32_t frame_start_ms = lv_tick_get();
  dashboard_data_t data;
  dashboard_demo_fill(&data, lv_tick_elaps(runtime->start_ms));
  dashboard_ui_set_data(runtime->ui, &data);

  const uint32_t frame_time_ms = lv_tick_elaps(frame_start_ms);
  if (frame_time_ms > runtime->fps_slowest_frame_ms) {
    runtime->fps_slowest_frame_ms = frame_time_ms;
  }
  runtime->fps_frame_count++;

  const uint32_t window_elapsed_ms = lv_tick_elaps(runtime->fps_window_start_ms);
  if (window_elapsed_ms >= 1000) {
    const uint32_t fps = (runtime->fps_frame_count * 1000U) / window_elapsed_ms;
    ESP_LOGI(TAG, "UI FPS=%lu (target>=10), slowest_frame=%lums, window=%lums",
             (unsigned long)fps, (unsigned long)runtime->fps_slowest_frame_ms,
             (unsigned long)window_elapsed_ms);
    runtime->fps_window_start_ms = lv_tick_get();
    runtime->fps_frame_count = 0;
    runtime->fps_slowest_frame_ms = 0;
  }
}

void app_main(void) {
  ESP_ERROR_CHECK(ws_display_init());
  ESP_ERROR_CHECK(ws_display_lock(-1));

  lv_obj_t *screen = lv_screen_active();
  s_runtime.ui = dashboard_ui_init(screen, ws_lcd_font_get_cb, NULL);
  if (s_runtime.ui == NULL) {
    ESP_LOGE(TAG, "Failed to create dashboard UI");
    ws_display_unlock();
    return;
  }

  s_runtime.start_ms = lv_tick_get();
  s_runtime.fps_window_start_ms = s_runtime.start_ms;
  s_runtime.fps_frame_count = 0;
  s_runtime.fps_slowest_frame_ms = 0;

  dashboard_data_t data;
  dashboard_demo_fill(&data, 0);
  dashboard_ui_set_data(s_runtime.ui, &data);

#if LV_USE_SYSMON && LV_USE_PERF_MONITOR
  lv_sysmon_show_performance(lv_display_get_default());
#endif

  /* Run UI updates at 20 Hz to keep display refresh safely above 10 FPS. */
  (void)lv_timer_create(dashboard_timer_cb, 50, &s_runtime);

  ws_display_unlock();
}
