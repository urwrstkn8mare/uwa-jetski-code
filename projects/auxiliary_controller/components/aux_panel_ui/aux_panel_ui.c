#include "aux_panel_ui.h"

#include "esp_err.h"
#include "can.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps.h"
#include "lvgl.h"
#include "rudder_pot.h"
#include "t_display_s3.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *TAG_UI = "aux_panel_ui";

static tdisplays3_handle_t s_board;
static lv_obj_t *s_lbl;

static bool wait_lvgl_display_lock_ms(uint32_t total_ms, uint32_t slice_ms) {
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

static void ui_timer(lv_timer_t *t) {
  (void)t;
  const unsigned pot = rudder_pot_get_last_pct();

  int32_t la = 0, lo = 0;
  int16_t sp = 0, hd = 0;
  uint8_t fix = 0;
  uint8_t gga_q = 0;
  gps_get_snapshot(&la, &lo, &sp, &hd, &fix, &gga_q);

  uint32_t ca = 0, cf = 0;
  can_get_tx_stats(&ca, &cf);

  can_bus_health_t cw = {0};
  can_get_bus_health(&cw);

  unsigned sp_dec = (unsigned)labs((long)(sp % 10));
  unsigned hd_frac = (unsigned)labs((long)(hd % 100));
  int hd_deg = hd / 100;

  gps_live_debug_t gd;
  gps_get_live_debug(&gd);
  char age_buf[20];
  if (gd.ms_since_last_uart_line == 0xffffffffu) {
    strlcpy(age_buf, "--", sizeof(age_buf));
  } else {
    snprintf(age_buf, sizeof(age_buf), "%" PRIu32 "ms", gd.ms_since_last_uart_line);
  }

  const char *can_label = (can_is_ready() && cw.controller_started) ? cw.state_label : "off";

  char buf[1024];
  snprintf(
      buf, sizeof(buf),
      "Pot %u%% (pin1)\n"
      "CAN %s T%u R%u b%" PRIu32 "\n"
      "txQ %" PRIu32 " fl%" PRIu32 "\n"
      "fx%u gq%u st%u\n"
      "lat%ld lon%ld\n"
      "spd%u.%u hdg%d.%02u\n"
      "----\n"
      "GPS %ubps rxB%" PRIu32 " Ln%" PRIu32 "\n"
      " ok%" PRIu32 " x%" PRIu32 " %s\n"
      "%.100s\n"
      "%.100s",
      pot, can_label, (unsigned)cw.tx_error_count,
      (unsigned)cw.rx_error_count, cw.bus_error_events, ca, cf, (unsigned)fix, (unsigned)gga_q,
      (unsigned)gd.sats_used_last_gga, (long)la, (long)lo, (unsigned)(sp / 10), sp_dec, hd_deg, hd_frac,
      (unsigned)gd.uart_baud, gd.uart_bytes_rx, gd.uart_lines_rx, gd.nmea_parse_ok, gd.nmea_parse_fail,
      age_buf, gd.last_sentence, gd.prev_sentence);

  if (s_lbl == NULL) {
    return;
  }
  if (tdisplays3_display_lock(120)) {
    lv_label_set_text(s_lbl, buf);
    tdisplays3_display_unlock();
  }
}

void aux_panel_ui_init(void) {
  ESP_ERROR_CHECK(tdisplays3_init(&s_board));
  if (!wait_lvgl_display_lock_ms(3000, 100)) {
    ESP_LOGE(TAG_UI, "LVGL display lock timeout — no label/timer");
    return;
  }

  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x101018), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

  s_lbl = lv_label_create(scr);
  lv_obj_set_style_text_color(s_lbl, lv_color_white(), LV_PART_MAIN);
  /* Montserrat 12: room for raw NMEA + UART stats on small panel */
  lv_obj_set_style_text_font(s_lbl, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_pad_all(s_lbl, 0, LV_PART_MAIN);
  lv_obj_set_width(s_lbl, lv_display_get_horizontal_resolution(NULL) - 4);
  lv_label_set_long_mode(s_lbl, LV_LABEL_LONG_WRAP);
  lv_obj_align(s_lbl, LV_ALIGN_TOP_LEFT, 3, 3);
  lv_label_set_text(s_lbl,
                    "Auxiliary controller\nInitialising CAN / GPS…");
  tdisplays3_display_unlock();

  lv_timer_t *tm = lv_timer_create(ui_timer, 200, NULL);
  if (tm == NULL) {
    ESP_LOGW(TAG_UI, "LVGL periodic timer failed");
  } else {
    ESP_LOGI(TAG_UI, "debug display ready (LVGL timer running)");
  }
}
