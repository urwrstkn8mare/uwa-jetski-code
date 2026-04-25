#include "ws_display.h"

#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "esp_lv_adapter_display.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "waveshare_rgb_lcd_port.h"
#include "ws_display_internal.h"

static const char *TAG = "ws_display";
static TaskHandle_t s_lock_owner = NULL;
static uint32_t s_lock_depth = 0;

static esp_lv_adapter_rotation_t get_rotation(void) {
  switch (CONFIG_WS_DISPLAY_ROTATION) {
  case 0:
    return ESP_LV_ADAPTER_ROTATE_0;
  case 90:
    return ESP_LV_ADAPTER_ROTATE_90;
  case 180:
    return ESP_LV_ADAPTER_ROTATE_180;
  case 270:
    return ESP_LV_ADAPTER_ROTATE_270;
  default:
    return ESP_LV_ADAPTER_ROTATE_180;
  }
}

esp_err_t ws_display_init(void) {
  if (esp_lv_adapter_is_initialized()) {
    return ESP_ERR_INVALID_STATE;
  }

  ws_display_font_reset();
  s_lock_owner = NULL;
  s_lock_depth = 0;

  /* Use triple full buffering so the rotated RGB path gets the 3 frame buffers
   * it requires. */
  const esp_lv_adapter_tear_avoid_mode_t tear_mode =
      ESP_LV_ADAPTER_TEAR_AVOID_MODE_TRIPLE_FULL;
  const esp_lv_adapter_rotation_t rotation = get_rotation();
  const uint8_t num_fbs =
      esp_lv_adapter_get_required_frame_buffer_count(tear_mode, rotation);

  esp_lcd_panel_handle_t panel = NULL;
  esp_err_t err = waveshare_esp32_s3_rgb_lcd_init(&panel, num_fbs);
  if (err != ESP_OK) {
    return err;
  }

  esp_lv_adapter_config_t adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG();
  adapter_cfg.task_stack_size = 32768;
  adapter_cfg.stack_in_psram = true;

  err = esp_lv_adapter_init(&adapter_cfg);
  if (err != ESP_OK) {
    (void)esp_lcd_panel_del(panel);
    return err;
  }

  esp_lv_adapter_display_config_t disp_cfg =
      ESP_LV_ADAPTER_DISPLAY_RGB_DEFAULT_CONFIG(panel, NULL, WS_DISPLAY_H_RES,
                                                WS_DISPLAY_V_RES, rotation);
  disp_cfg.tear_avoid_mode = tear_mode;
  disp_cfg.profile.buffer_height = CONFIG_EXAMPLE_LCD_RGB_BOUNCE_BUFFER_HEIGHT;
  disp_cfg.profile.use_psram = true;
  disp_cfg.profile.require_double_buffer = false;

  lv_display_t *disp = esp_lv_adapter_register_display(&disp_cfg);
  if (disp == NULL)
    goto fail;

  if (esp_lv_adapter_start() != ESP_OK)
    goto fail;

  (void)disp;
  ESP_LOGI(TAG, "Display initialised");
  return ESP_OK;

fail:
  (void)esp_lv_adapter_deinit();
  (void)esp_lcd_panel_del(panel);
  return ESP_FAIL;
}

esp_err_t ws_display_lock(int32_t timeout_ms) {
  esp_err_t err = esp_lv_adapter_lock(timeout_ms);
  if (err != ESP_OK) {
    return err;
  }

  TaskHandle_t current = xTaskGetCurrentTaskHandle();
  if (s_lock_depth == 0) {
    s_lock_owner = current;
  }
  s_lock_depth++;
  return ESP_OK;
}

void ws_display_unlock(void) {
  TaskHandle_t current = xTaskGetCurrentTaskHandle();
  if (s_lock_depth == 0 || s_lock_owner != current) {
    ESP_LOGE(TAG,
             "ws_display_unlock() called without matching ws_display_lock()");
    return;
  }

  esp_lv_adapter_unlock();

  s_lock_depth--;
  if (s_lock_depth == 0) {
    s_lock_owner = NULL;
  }
}

bool ws_display_lock_held(void) {
  return s_lock_depth > 0 && s_lock_owner == xTaskGetCurrentTaskHandle();
}
