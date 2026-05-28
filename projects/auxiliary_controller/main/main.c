#include "can.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "gps.h"
#include "joystick.h"
#include "status_ui.h"
#include "t_display_s3.h"

#include "lvgl.h"

static const char *TAG = "aux_main";

void app_main(void) {
  tdisplays3_handle_t disp_board;
  if (tdisplays3_init(&disp_board) == ESP_OK) {
    if (tdisplays3_display_lock(200)) {
      lv_obj_t *scr = lv_screen_active();
      lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), LV_PART_MAIN);
      lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
      tdisplays3_display_unlock();
    }
    const status_ui_cfg_t ui_cfg = {
        .parent          = lv_screen_active(),
        .lock_cb         = tdisplays3_display_lock,
        .unlock_cb       = tdisplays3_display_unlock,
        .lock_timeout_ms = 200,
        .min_interval_ms = 200,
    };
    if (status_ui_start(&ui_cfg) != ESP_OK) {
      ESP_LOGW(TAG, "status_ui_start failed — serial only");
    }
  } else {
    ESP_LOGW(TAG, "tdisplays3_init failed — serial only");
  }

  if (can_init() != ESP_OK) {
    ESP_LOGW(TAG, "CAN init failed — CAN TX disabled");
  }

  if (gps_init() != ESP_OK) ESP_LOGW(TAG, "GPS not initialised");
  if (joystick_init() != ESP_OK) {
    ESP_LOGW(TAG, "Joystick ADC not initialised");
  }
}
