#include "bsp/m5stack_tab5.h"
#include "can.h"
#include "can_ui_bridge.h"
#include "dashboard_demo.h"
#include "dashboard_ui.h"
#include "esp_log.h"
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
  esp_io_expander_handle_t io_expander = bsp_io_expander_init();

  if (io_expander != NULL) {
    esp_err_t ret = esp_io_expander_set_output_mode(io_expander, IO_EXPANDER_PIN_NUM_2, IO_EXPANDER_OUTPUT_MODE_PUSH_PULL);
    if (ret == ESP_OK) {
      ret = esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_2, 1);
    }
    if (ret == ESP_OK) {
      ESP_LOGI(TAG, "External 5V (EXT5V) enabled successfully");
    } else {
      ESP_LOGE(TAG, "Failed to enable external 5V (EXT5V): %s", esp_err_to_name(ret));
    }
  } else {
    ESP_LOGE(TAG, "IO expander not initialized, cannot enable EXT5V");
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
  bsp_display_rotate(disp, LV_DISPLAY_ROTATION_270);
  bsp_display_backlight_on();

  bsp_feature_enable(BSP_FEATURE_SPEAKER, false);
  bsp_feature_enable(BSP_FEATURE_CAMERA, false);
  bsp_feature_enable(BSP_FEATURE_USB, false);
  bsp_feature_enable(BSP_FEATURE_WIFI, false);
  ESP_LOGI(TAG, "Unused peripherals (speaker, camera, USB, WiFi) powered off");

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
  ESP_LOGI(TAG, "CAN initialised, waiting for data...");

  (void)lv_timer_create(dashboard_timer_cb, 50, &s_runtime);
  bsp_display_unlock();
}
