#include "main_panel_ui.h"

#include "app_state.h"
#include "can.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "height.h"
#include "imu.h"
#include "lvgl.h"
#include "servo_drive.h"
#include "t_display_s3.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

static const char *TAG_UI = "main_panel_ui";

static tdisplays3_handle_t s_board;
static lv_obj_t *s_dbg_label;

/* taskLVGL runs this timer only on that task — static scratch avoids stack churn. */
static char s_pr_line[56];
static char s_h_line[40];
static char s_servo_line[40];
static char s_can_line[64];
static char s_ui_buf[320];

static void ui_refresh_timer(lv_timer_t *t) {
  (void)t;
  app_state_t rh;
  app_state_get(&rh);

  float pitch = 0, roll = 0, yaw = 0;
  if (rh.imu_ok && imu_get_pitch_roll_yaw(&pitch, &roll, &yaw) == ESP_OK) {
    snprintf(s_pr_line, sizeof(s_pr_line), "P:%.1f R:%.1f Y:%.1f deg", (double)pitch, (double)roll, (double)yaw);
  } else {
    strlcpy(s_pr_line, "P/R/Y: --- (IMU off)", sizeof(s_pr_line));
  }

  int32_t hcm = -1;
  if (rh.height_ok && height_get_cm(&hcm) == ESP_OK) {
    snprintf(s_h_line, sizeof(s_h_line), "H:%" PRId32 " cm", hcm);
  } else {
    strlcpy(s_h_line, rh.height_ok ? "H: --- (no reading)" : "H: off (ultrasonic N/C)",
            sizeof(s_h_line));
  }

  uint16_t pot = 50;
  const bool pot_fresh = app_state_pot_fresh(500, &pot);
  if (rh.can_ok && can_is_ready()) {
    (void)can_snprintf_board_status(s_can_line, sizeof(s_can_line));
  } else {
    snprintf(s_can_line, sizeof(s_can_line),
             "CAN off Tx%u Rx%u (init skipped / TWAI down)",
             (unsigned)CONFIG_CANTX, (unsigned)CONFIG_CANRX);
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
  if (tdisplays3_display_lock(100)) {
    lv_label_set_text(s_dbg_label, s_ui_buf);
    tdisplays3_display_unlock();
  }
}

void main_panel_ui_init(void) {
  app_state_set_display(false);
  if (tdisplays3_init(&s_board) != ESP_OK) {
    ESP_LOGW(TAG_UI, "tdisplays3_init failed — serial only");
    return;
  }
  if (s_board.display == NULL) {
    ESP_LOGW(TAG_UI, "Display handle null — serial only");
    return;
  }
  if (!tdisplays3_display_lock(200)) {
    ESP_LOGW(TAG_UI, "LVGL display lock timeout — serial only");
    return;
  }
  app_state_set_display(true);

  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x101018), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

  s_dbg_label = lv_label_create(scr);
  if (s_dbg_label == NULL) {
    ESP_LOGW(TAG_UI, "Label create failed — display online without debug text");
    tdisplays3_display_unlock();
    return;
  }
  lv_obj_set_style_text_color(s_dbg_label, lv_color_white(), LV_PART_MAIN);
  /* Keep default font as fallback if montserrat_20 is not linked. */
  lv_obj_set_style_text_font(s_dbg_label, LV_FONT_DEFAULT, LV_PART_MAIN);
  {
    const lv_coord_t hres = lv_display_get_horizontal_resolution(s_board.display);
    lv_obj_set_width(s_dbg_label, hres > 4 ? hres - 4 : hres);
  }
  lv_label_set_long_mode(s_dbg_label, LV_LABEL_LONG_WRAP);
  lv_obj_align(s_dbg_label, LV_ALIGN_TOP_LEFT, 4, 4);
  lv_label_set_text(s_dbg_label, "MAIN controller\nBooting...");
  tdisplays3_display_unlock();
  ESP_LOGI(TAG_UI, "Display UI initialized");

  lv_timer_t *upd = lv_timer_create(ui_refresh_timer, 200, NULL);
  if (upd == NULL) {
    ESP_LOGW(TAG_UI, "LVGL timer create failed — static text only");
  }
}
