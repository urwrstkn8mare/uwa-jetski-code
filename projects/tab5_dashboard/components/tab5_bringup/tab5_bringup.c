#include "tab5_bringup.h"

#include "bsp/m5stack_tab5.h"
#include "esp_log.h"
#include "esp_io_expander.h"

static const char *TAG = "tab5_bringup";

esp_err_t tab5_bringup_init(lv_display_t **out_display) {
  if (out_display == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_io_expander_handle_t io_expander = bsp_io_expander_init();
  if (io_expander != NULL) {
    esp_err_t ret = esp_io_expander_set_output_mode(io_expander, IO_EXPANDER_PIN_NUM_2, IO_EXPANDER_OUTPUT_MODE_PUSH_PULL);
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
  bsp_display_rotate(disp, LV_DISPLAY_ROTATION_270);
  bsp_display_backlight_on();

  bsp_feature_enable(BSP_FEATURE_SPEAKER, false);
  bsp_feature_enable(BSP_FEATURE_CAMERA, false);
  bsp_feature_enable(BSP_FEATURE_USB, false);
  bsp_feature_enable(BSP_FEATURE_WIFI, false);
  ESP_LOGI(TAG, "Unused blocks (speaker, camera, USB, WiFi) off");

  *out_display = disp;
  return ESP_OK;
}
