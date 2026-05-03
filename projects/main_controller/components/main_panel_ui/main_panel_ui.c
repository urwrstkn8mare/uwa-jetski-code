#include "main_panel_ui.h"

#include "esp_err.h"
#include "height.h"
#include "imu.h"
#include "lvgl.h"
#include "t_display_s3.h"
#include "vehicle_inputs.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>

static tdisplays3_handle_t s_board;
static lv_obj_t *s_dbg_label;

static void ui_refresh_timer(lv_timer_t *t) {
  (void)t;
  float pitch = 0, roll = 0;
  (void)imu_get_pitch_roll(&pitch, &roll);
  int32_t hcm = -1;
  (void)height_get_cm(&hcm);

  uint16_t pot = vehicle_inputs_get_pot_pct();
  int32_t lat = 0, lon = 0;
  int16_t speed_x10 = 0;
  int16_t hdg_c = 0;
  bool gps = false;
  (void)vehicle_inputs_get_gps(&lat, &lon, &speed_x10, &hdg_c, &gps);

  char buf[512];
  snprintf(buf, sizeof(buf),
           "MAIN node\n"
           "P:%.1f R:%.1f deg\n"
           "H:%" PRId32 " cm\n"
           "Rudder demand %u%% → pwm %lu µs\n"
           "lat_e7:%" PRId32 " lon_e7:%" PRId32 "\n"
           "spd:%d.%d km/h hdg:%d.%02d deg\n"
           "%s",
           (double)pitch, (double)roll, hcm, (unsigned)pot,
           (unsigned long)(1000u + (unsigned)pot * 10u), lat, lon, speed_x10 / 10,
           abs(speed_x10 % 10), hdg_c / 100, abs(hdg_c % 100),
           gps ? "GPS ok" : "no GPS fix pos");

  if (tdisplays3_display_lock(100)) {
    lv_label_set_text(s_dbg_label, buf);
    tdisplays3_display_unlock();
  }
}

void main_panel_ui_init(void) {
  ESP_ERROR_CHECK(tdisplays3_init(&s_board));
  if (!tdisplays3_display_lock(200)) {
    return;
  }
  s_dbg_label = lv_label_create(lv_screen_active());
  lv_obj_set_width(s_dbg_label, lv_display_get_horizontal_resolution(NULL) - 8);
  lv_label_set_long_mode(s_dbg_label, LV_LABEL_LONG_WRAP);
  lv_obj_align(s_dbg_label, LV_ALIGN_TOP_LEFT, 4, 4);
  lv_label_set_text(s_dbg_label, "MAIN\ninit…");
  tdisplays3_display_unlock();
  (void)lv_timer_create(ui_refresh_timer, 200, NULL);
}
