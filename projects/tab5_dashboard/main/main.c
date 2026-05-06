/* Tab5 dashboard: CAN/DEMO mode wiring. */

#include "bsp/m5stack_tab5.h"
#include "dashboard_can.h"
#include "dashboard_demo.h"
#include "dashboard_ui.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "lvgl.h"
#include "lvgl_status_display.h"
#include "sdkconfig.h"
#include "esp_io_expander.h"
#include "ui/dashboard_font.h"

#include <stdint.h>
#include <stdio.h>

static const char *TAG = "tab5_main";

static dashboard_ui_t *s_ui;
static lvgl_status_display_t s_strip;

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

  const int hz = CONFIG_TAB5_DASHBOARD_DEMO_HZ;
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

  s_ui = dashboard_ui_init(dashboard_host, tab5_font_get_cb, NULL);
  if (s_ui == NULL) {
    ESP_LOGE(TAG, "Failed to initialise dashboard_ui");
    bsp_display_unlock();
    return;
  }

  lv_obj_t *strip_label = dashboard_ui_create_status_strip(screen, h_res, strip_h_px);
  if (strip_label == NULL) {
    ESP_LOGE(TAG, "Failed to create status strip label");
    bsp_display_unlock();
    return;
  }

#if CONFIG_TAB5_DASHBOARD_FEED_MODE_DEMO
  s_demo_ctx.ui = s_ui;
  s_demo_ctx.start_ms = lv_tick_get();

  const uint32_t demo_hz = (CONFIG_TAB5_DASHBOARD_DEMO_HZ > 0) ? (uint32_t)CONFIG_TAB5_DASHBOARD_DEMO_HZ : 1u;
  const uint32_t demo_period_ms = (1000u + (demo_hz / 2u)) / demo_hz;

  (void)lv_timer_create(demo_timer_cb, demo_period_ms, &s_demo_ctx);

  dashboard_demo_update_at(s_ui, 0);

  lvgl_status_line_t lines[1] = {
      {.write = demo_status_strip_write, .ctx = NULL},
  };
  ESP_ERROR_CHECK(lvgl_status_display_start(&s_strip, strip_label, lines, 1, 250, 64));
#else
  ESP_ERROR_CHECK(dashboard_can_attach(s_ui, tab5_can_lock, tab5_can_unlock, NULL));
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
  ESP_ERROR_CHECK(lvgl_status_display_start(&s_strip, strip_label, lines, 1, 250, 96));
#endif

  bsp_display_unlock();
}
