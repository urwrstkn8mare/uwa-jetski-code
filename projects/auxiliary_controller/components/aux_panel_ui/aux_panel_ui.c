#include "aux_panel_ui.h"

#include "esp_err.h"
#include "can.h"
#include "esp_log.h"
#include "gps.h"
#include "lvgl.h"
#include "rudder_pot.h"
#include "t_display_s3.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>

static tdisplays3_handle_t s_board;
static lv_obj_t *s_lbl;

static void ui_timer(lv_timer_t *t) {
  (void)t;
  const unsigned pot = rudder_pot_get_last_pct();

  int32_t la = 0, lo = 0;
  int16_t sp = 0, hd = 0;
  uint8_t fix = 0;
  gps_get_snapshot(&la, &lo, &sp, &hd, &fix);

  uint32_t ca = 0, cf = 0;
  can_get_tx_stats(&ca, &cf);

  char buf[512];
  snprintf(buf, sizeof(buf),
           "Auxiliary controller\n"
           "Rudder TX %u%%\n"
           "GPS fix:%u lat_e7:%ld lon_e7:%ld\n"
           "spd:%d.%d km/h hdg:%d.%02d\n"
           "CAN tx try:%" PRIu32 " fail:%" PRIu32 "",
           pot, (unsigned)fix, (long)la, (long)lo, sp / 10, abs(sp % 10), hd / 100, abs(hd % 100),
           ca, cf);

  if (tdisplays3_display_lock(120)) {
    lv_label_set_text(s_lbl, buf);
    tdisplays3_display_unlock();
  }
}

void aux_panel_ui_init(void) {
  ESP_ERROR_CHECK(tdisplays3_init(&s_board));
  if (!tdisplays3_display_lock(200)) {
    return;
  }
  ESP_LOGI("debug_ui", "ui started");
  s_lbl = lv_label_create(lv_screen_active());
  lv_obj_set_width(s_lbl, lv_display_get_horizontal_resolution(NULL) - 6);
  lv_label_set_long_mode(s_lbl, LV_LABEL_LONG_WRAP);
  lv_obj_align(s_lbl, LV_ALIGN_TOP_LEFT, 3, 3);
  lv_label_set_text(s_lbl, "Auxiliary…");
  tdisplays3_display_unlock();
  (void)lv_timer_create(ui_timer, 200, NULL);
}
