#include "main_panel_ui.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "can.h"
#include "height.h"
#include "imu.h"
#include "lvgl.h"
#include "runtime_health.h"
#include "t_display_s3.h"
#include "servo_drive.h"
#include "vehicle_inputs.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

static const char *TAG_UI = "main_panel_ui";

static lv_disp_t *s_display;
static lv_obj_t *s_dbg_label;

/* taskLVGL runs this timer only on that task — static scratch avoids stack churn. */
static char s_pr_line[40];
static char s_h_line[40];
static char s_servo_line[40];
static char s_can_line[64];
static char s_ui_buf[320];

static void ui_refresh_timer(lv_timer_t *t) {
  (void)t;
  runtime_health_t rh;
  runtime_health_get(&rh);

  float pitch = 0, roll = 0;
  if (rh.imu_ok && imu_get_pitch_roll(&pitch, &roll) == ESP_OK) {
    snprintf(s_pr_line, sizeof(s_pr_line), "P:%.1f R:%.1f deg", (double)pitch, (double)roll);
  } else {
    strlcpy(s_pr_line, "P/R: --- (IMU off)", sizeof(s_pr_line));
  }

  int32_t hcm = -1;
  if (rh.height_ok && height_get_cm(&hcm) == ESP_OK) {
    snprintf(s_h_line, sizeof(s_h_line), "H:%" PRId32 " cm", hcm);
  } else {
    strlcpy(s_h_line, rh.height_ok ? "H: --- (no reading)" : "H: off (ultrasonic N/C)",
            sizeof(s_h_line));
  }

  uint16_t pot = 50;
  const bool pot_fresh = vehicle_inputs_get_pot_fresh(500, &pot);
  can_bus_health_t bh = {0};
  uint32_t ca = 0, cf = 0;
  can_get_tx_stats(&ca, &cf);
  if (rh.can_ok && can_is_ready()) {
    can_get_bus_health(&bh);
    snprintf(s_can_line, sizeof(s_can_line), "CAN %s T%u R%u b%" PRIu32 " q%" PRIu32 " f%" PRIu32,
             bh.state_label, (unsigned)bh.tx_error_count, (unsigned)bh.rx_error_count, bh.bus_error_events, ca, cf);
  } else {
    strlcpy(s_can_line, "CAN off T0 R0 b0 q0 f0", sizeof(s_can_line));
  }

  const char *imu_state = rh.imu_ok ? "ok" : "off";
  const char *height_state = rh.height_ok ? "ok" : "off";
  const char *servo_state = (rh.servo_ok && servo_drive_is_ready()) ? "ok" : "off";
  uint32_t mch0_us = 0;
  uint32_t mch1_us = 0;
  if (rh.servo_ok && servo_drive_is_ready()) {
    servo_drive_get_pulse_us(&mch0_us, &mch1_us);
    snprintf(s_servo_line, sizeof(s_servo_line), "S0:%" PRIu32 " S1:%" PRIu32, mch0_us, mch1_us);
  } else {
    strlcpy(s_servo_line, "Servo: off", sizeof(s_servo_line));
  }

  snprintf(s_ui_buf, sizeof(s_ui_buf),
           "MAIN controller\n"
           "Display:%s\n"
           "%s\n"
           "IMU:%s Height:%s Servo:%s\n"
           "%s\n"
           "%s\n"
           "%s\n"
           "Rudder: %s %u%%",
           rh.display_ok ? "ok" : "off", s_can_line, imu_state, height_state, servo_state, s_pr_line, s_h_line,
           s_servo_line, pot_fresh ? "CAN" : "DEMO", (unsigned)pot);

  if (s_dbg_label == NULL) {
    return;
  }
  if (lvgl_port_lock(100)) {
    lv_label_set_text(s_dbg_label, s_ui_buf);
    lvgl_port_unlock();
  }
}

void main_panel_ui_init(void) {
  runtime_health_set_display(false);
  lcd_init(&s_display, true);
  if (s_display == NULL) {
    ESP_LOGW(TAG_UI, "Display init failed — serial only");
    return;
  }
  if (!lvgl_port_lock(200)) {
    ESP_LOGW(TAG_UI, "LVGL display lock timeout — serial only");
    return;
  }
  runtime_health_set_display(true);

  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x101018), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

  s_dbg_label = lv_label_create(scr);
  if (s_dbg_label == NULL) {
    ESP_LOGW(TAG_UI, "Label create failed — display online without debug text");
    lvgl_port_unlock();
    return;
  }
  lv_obj_set_style_text_color(s_dbg_label, lv_color_white(), LV_PART_MAIN);
  /* Keep default font as fallback if montserrat_20 is not linked. */
  lv_obj_set_style_text_font(s_dbg_label, LV_FONT_DEFAULT, LV_PART_MAIN);
  lv_obj_set_width(s_dbg_label, 312);
  lv_label_set_long_mode(s_dbg_label, LV_LABEL_LONG_WRAP);
  lv_obj_align(s_dbg_label, LV_ALIGN_TOP_LEFT, 4, 4);
  lv_label_set_text(s_dbg_label, "MAIN controller\nBooting...");
  lvgl_port_unlock();
  ESP_LOGI(TAG_UI, "Display UI initialized");

  lv_timer_t *upd = lv_timer_create(ui_refresh_timer, 200, NULL);
  if (upd == NULL) {
    ESP_LOGW(TAG_UI, "LVGL timer create failed — static text only");
  }
}
