#include "can.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps.h"
#include "status_ui.h"
#include "rudder_pot.h"
#include "t_display_s3.h"

#include "lvgl.h"

#include <stdint.h>
#include <stdio.h>

static const char *TAG = "aux_main";

static tdisplays3_handle_t s_tdisp_board;
static status_ui_t s_status_ui;

static void aux_status_lock(void) { tdisplays3_display_lock(200); }

static void aux_status_unlock(void) { tdisplays3_display_unlock(); }

static void auxiliary_status_display_init(void) {
  if (tdisplays3_init(&s_tdisp_board) != ESP_OK) {
    ESP_LOGW(TAG, "tdisplays3_init failed");
    return;
  }
  if (!tdisplays3_display_lock(3000)) {
    ESP_LOGE(TAG, "LVGL display lock timeout — no label/timer");
    return;
  }

  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x101018), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

  lv_obj_t *panel = lv_obj_create(scr);
  lv_obj_remove_style_all(panel);
  lv_obj_set_size(panel, LV_PCT(100), LV_PCT(100));
  lv_obj_set_style_bg_color(panel, lv_color_hex(0x000000), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_align(panel, LV_ALIGN_CENTER, 0, 0);

  const status_ui_cfg_t cfg = {
      .parent = panel,
      .lock_cb = aux_status_lock,
      .unlock_cb = aux_status_unlock,
      .min_interval_ms = 200,
  };
  if (status_ui_start(&s_status_ui, &cfg) != ESP_OK) {
    ESP_LOGW(TAG, "status display init failed");
  }

  tdisplays3_display_unlock();
}

static void aux_task_loop(void *arg) {
  status_ui_t *disp = (status_ui_t *)arg;
  for (;;) {
    if (disp != NULL) {
      if (can_is_ready()) {
        char buf[128];
        int n = can_snprintf_board_status(buf, sizeof(buf));
        if (n > 0) {
          status_ui_update(disp, "CAN", "%s", buf);
        }
      } else {
        status_ui_update(disp, "CAN", "CAN off (TWAI down)");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void app_main(void) {
  auxiliary_status_display_init();

  if (can_init(NULL) != ESP_OK) {
    ESP_LOGW(TAG, "CAN init failed — display still active; TX disabled until fixed");
  }

  gps_init(status_ui_update, &s_status_ui);
  if (rudder_pot_init(status_ui_update, &s_status_ui) != ESP_OK) {
    ESP_LOGW(TAG, "Rudder ADC not initialised");
  }

  xTaskCreate(aux_task_loop, "aux_io", 2048, &s_status_ui, 2, NULL);
}
