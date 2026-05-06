/*
 * Auxiliary controller (T-Display-S3)
 *
 * - Rudder potentiometer → CAN (demand %) — detailed attitude/servos/GPS HUD live on main + dashboard projects.
 * - Neo-6M GPS → CAN position/velocity + local GPS status on this display.
 */

#include "can.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps.h"
#include "lvgl_status_display.h"
#include "rudder_pot.h"
#include "t_display_s3.h"

#include "lvgl.h"

#include <stdint.h>
#include <stdio.h>

static const char *TAG = "aux_main";

static tdisplays3_handle_t s_tdisp_board;
static lvgl_status_display_t s_status_dsp;

static esp_err_t init_retry(const char *name, esp_err_t (*fn)(void), int n) {
  esp_err_t r = ESP_FAIL;
  for (int i = 0; i < n; i++) {
    r = fn();
    if (r == ESP_OK) {
      return ESP_OK;
    }
    ESP_LOGW(TAG, "%s init fail (%d/%d): %s", name, i + 1, n, esp_err_to_name(r));
    vTaskDelay(pdMS_TO_TICKS(400));
  }
  return r;
}

static esp_err_t can_init_retry(void) { return can_init(NULL); }

static bool aux_wait_lvgl_display_lock_ms(uint32_t total_ms, uint32_t slice_ms) {
  uint32_t waited = 0;
  while (waited <= total_ms) {
    uint32_t t = slice_ms;
    if (waited + t > total_ms) {
      t = total_ms - waited;
    }
    if (t == 0) {
      t = slice_ms ? slice_ms : 50;
    }
    if (tdisplays3_display_lock(t)) {
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    waited += slice_ms;
  }
  return false;
}

static size_t aux_line_rudder(char *buf, size_t cap, void *ctx) {
  (void)ctx;
  return rudder_pot_status_line_write(buf, cap);
}

static size_t aux_line_can(char *buf, size_t cap, void *ctx) {
  (void)ctx;
  if (can_is_ready()) {
    int n = can_snprintf_board_status(buf, cap);
    return (n > 0) ? (size_t)n : 0;
  }
  int n = snprintf(buf, cap, "CAN off (TWAI down)");
  return (n > 0) ? (size_t)n : 0;
}

static size_t aux_line_gps(char *buf, size_t cap, void *ctx) {
  (void)ctx;
  return gps_status_write(buf, cap);
}

static const lvgl_status_line_t AUX_STATUS_LINES[] = {
    {.write = aux_line_rudder, .ctx = NULL},
    {.write = aux_line_can, .ctx = NULL},
    {.write = aux_line_gps, .ctx = NULL},
};

static void auxiliary_status_display_init(void) {
  if (tdisplays3_init(&s_tdisp_board) != ESP_OK) {
    ESP_LOGW(TAG, "tdisplays3_init failed");
    return;
  }
  if (!aux_wait_lvgl_display_lock_ms(3000, 100)) {
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

  lv_obj_t *lbl = lv_label_create(panel);
  lv_obj_set_style_text_color(lbl, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_pad_all(lbl, 0, LV_PART_MAIN);
  lv_coord_t hres = lv_display_get_horizontal_resolution(s_tdisp_board.display);
  lv_obj_set_width(lbl, hres > 4 ? hres - 4 : hres);
  lv_label_set_long_mode(lbl, LV_LABEL_LONG_WRAP);
  lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 3, 3);
  lv_obj_move_foreground(panel);
  lv_obj_move_foreground(lbl);
  lv_label_set_text(lbl, "Auxiliary controller\nInitialising…");
  tdisplays3_display_unlock();

  const size_t nlines = sizeof(AUX_STATUS_LINES) / sizeof(AUX_STATUS_LINES[0]);
  if (lvgl_status_display_start(&s_status_dsp, lbl, AUX_STATUS_LINES, nlines, 200, 1536) != ESP_OK) {
    ESP_LOGW(TAG, "status display timer failed");
  }
}

void app_main(void) {
  auxiliary_status_display_init();

  if (init_retry("CAN", can_init_retry, 3) != ESP_OK) {
    ESP_LOGW(TAG, "CAN init failed — display still active; TX disabled until fixed");
  }

  gps_init();
  if (rudder_pot_init() != ESP_OK) {
    ESP_LOGW(TAG, "Rudder ADC not initialised");
  }
}
