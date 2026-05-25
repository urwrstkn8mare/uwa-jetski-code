/* Tab5 dashboard: CAN/DEMO mode wiring. */

#include "bsp/m5stack_tab5.h"
#include "dashboard_can.h"
#include "dashboard_demo.h"
#include "dashboard_ui.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "lvgl.h"
#include "status_ui.h"
#include "sdkconfig.h"
#include "esp_io_expander.h"
#include "ui/dashboard_font.h"

#include <stdint.h>
#include <stdio.h>

static const char *TAG = "tab5_main";

extern int FT_Stream_Open(void *stream, const char *filepathname);
static void *__attribute__((used, section(".data"))) force_lv_ftsystem_link_ptr =
    (void *)FT_Stream_Open;

static esp_err_t tab5_board_bringup(lv_display_t **out_display) {
  if (out_display == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_io_expander_handle_t io_expander = bsp_io_expander_init();
  if (io_expander != NULL) {
    esp_err_t ret = esp_io_expander_set_output_mode(io_expander, IO_EXPANDER_PIN_NUM_2,
                                                      IO_EXPANDER_OUTPUT_MODE_PUSH_PULL);
    if (ret == ESP_OK) {
      ret = esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_2, 1);
    }
    if (ret == ESP_OK) {
      ESP_LOGI(TAG, "External 5V (EXT5V) enabled");
    } else {
      ESP_LOGE(TAG, "EXT5V: %s", esp_err_to_name(ret));
    }
  } else {
    ESP_LOGE(TAG, "IO expander not initialised, cannot enable EXT5V");
  }

  bsp_display_cfg_t cfg = {
      .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
      .buffer_size = BSP_LCD_H_RES * CONFIG_BSP_LCD_DRAW_BUF_HEIGHT,
      .double_buffer = true,
      .flags = {
          .buff_dma = 1,
          .buff_spiram = 0,
          .sw_rotate = 1,
      },
  };
  cfg.lvgl_port_cfg.task_stack = 24576;

  lv_display_t *disp = bsp_display_start_with_config(&cfg);
  bsp_display_rotate(disp, LV_DISPLAY_ROTATION_90);
  bsp_display_backlight_on();

  bsp_feature_enable(BSP_FEATURE_SPEAKER, false);
  bsp_feature_enable(BSP_FEATURE_CAMERA, false);
  bsp_feature_enable(BSP_FEATURE_USB, false);
  bsp_feature_enable(BSP_FEATURE_WIFI, false);
  bsp_feature_enable(BSP_FEATURE_TOUCH, false);

  ESP_LOGI(TAG, "Unused blocks (speaker, camera, USB, WiFi, touch) off");
  *out_display = disp;
  return ESP_OK;
}

static const lv_font_t *tab5_font_get_cb(uint16_t size_px, int weight, void *user_data) {
  (void)user_data;
  const lv_font_t *font = NULL;
  if (dashboard_font_get(size_px, weight, &font) == ESP_OK) {
    return font;
  }
  return NULL;
}

#if !CONFIG_TAB5_DASHBOARD_FEED_MODE_DEMO
static esp_err_t tab5_can_lock(int32_t timeout_ms, void *ctx) {
  (void)ctx;
  const uint32_t timeout = (timeout_ms < 0) ? (uint32_t)portMAX_DELAY : (uint32_t)timeout_ms;
  return bsp_display_lock(timeout) ? ESP_OK : ESP_FAIL;
}

static void tab5_can_unlock(void *ctx) {
  (void)ctx;
  bsp_display_unlock();
}
#endif

#if CONFIG_TAB5_DASHBOARD_FEED_MODE_DEMO
typedef struct {
  uint32_t start_ms;
} demo_ctx_t;

static demo_ctx_t s_demo_ctx;

static void demo_status_timer_cb(lv_timer_t *timer) {
  (void)timer;
  const int hz = (CONFIG_TAB5_DASHBOARD_DEMO_HZ > 0) ? CONFIG_TAB5_DASHBOARD_DEMO_HZ : 1;
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
  lv_display_t *disp = NULL;
  if (tab5_board_bringup(&disp) != ESP_OK) {
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
  const int32_t h_res = default_disp ? (int32_t)lv_display_get_horizontal_resolution(default_disp)
                                      : 1280;
  const int32_t v_res = default_disp ? (int32_t)lv_display_get_vertical_resolution(default_disp)
                                      : 720;

  const int32_t strip_h_px = 28;
  const int32_t dashboard_h = v_res - strip_h_px;
  if (dashboard_h <= 0) {
    ESP_LOGE(TAG, "Invalid dashboard area: v_res=%d strip_h=%d", (int)v_res, (int)strip_h_px);
    bsp_display_unlock();
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
      .font_get_cb = tab5_font_get_cb,
      .font_get_user_data = NULL,
      .lock_cb = bsp_display_lock,
      .unlock_cb = bsp_display_unlock,
      .lock_timeout_ms = portMAX_DELAY,
  };
  esp_err_t ui_err = dashboard_ui_init(&ui_cfg);
  if (ui_err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialise dashboard_ui: %s", esp_err_to_name(ui_err));
    bsp_display_unlock();
    return;
  }

#if CONFIG_TAB5_DASHBOARD_FEED_MODE_DEMO
  s_demo_ctx.start_ms = lv_tick_get();

  const uint32_t demo_hz = (CONFIG_TAB5_DASHBOARD_DEMO_HZ > 0) ? (uint32_t)CONFIG_TAB5_DASHBOARD_DEMO_HZ : 1u;
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
      .lock_cb = bsp_display_lock,
      .unlock_cb = bsp_display_unlock,
      .lock_timeout_ms = portMAX_DELAY,
      .min_interval_ms = 250,
  };
  ESP_ERROR_CHECK(status_ui_start(&cfg));

  (void)lv_timer_create(demo_status_timer_cb, 250, NULL);
#else
  ESP_ERROR_CHECK(dashboard_can_attach(tab5_can_lock, tab5_can_unlock, NULL));

  const status_ui_cfg_t cfg = {
      .parent = screen,
      .flex_flow = LV_FLEX_FLOW_ROW,
      .w = h_res,
      .h = strip_h_px,
      .align = LV_ALIGN_BOTTOM_MID,
      .bg_opa = LV_OPA_COVER,
      .lock_cb = bsp_display_lock,
      .unlock_cb = bsp_display_unlock,
      .lock_timeout_ms = portMAX_DELAY,
      .min_interval_ms = 250,
  };
  ESP_ERROR_CHECK(status_ui_start(&cfg));

  esp_err_t can_err = dashboard_can_start();
  if (can_err != ESP_OK) {
    ESP_LOGW(TAG, "CAN start failed: %s (UI will stay up)", esp_err_to_name(can_err));
    status_ui_update("CAN", "off (TWAI down)");
  }
#endif

  bsp_display_unlock();
}
