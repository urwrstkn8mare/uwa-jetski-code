/*
 * Main controller (T-Display-S3): debug display + servos + CAN + height + IMU.
 */

#include "app_state.h"
#include "can.h"
#include "can_ids.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "height.h"
#include "imu.h"
#include "lvgl_status_display.h"
#include "servo_drive.h"
#include "t_display_s3.h"

#include "lvgl.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "main";

enum { kTaskPeriodMs = 50 };
enum { kWorkTaskPrio = 2 };

static tdisplays3_handle_t s_tdisp_board;
static lvgl_status_display_t s_status_dsp;

static void on_can_rx(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp) {
  (void)timestamp;
  app_state_on_can_rx(buffer, header_id);
}

static esp_err_t can_start(void) { return can_init(on_can_rx); }

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

static size_t main_line_title(char *buf, size_t cap, void *ctx) {
  (void)ctx;
  int n = snprintf(buf, cap, "MAIN controller");
  return (n > 0) ? (size_t)n : 0;
}

static size_t main_line_display(char *buf, size_t cap, void *ctx) {
  (void)ctx;
  app_state_t s;
  app_state_get(&s);
  int n = snprintf(buf, cap, "Display:%s", s.display_ok ? "ok" : "off");
  return (n > 0) ? (size_t)n : 0;
}

static size_t main_line_can(char *buf, size_t cap, void *ctx) {
  (void)ctx;
  app_state_t st;
  app_state_get(&st);
  if (st.can_ok && can_is_ready()) {
    int n = can_snprintf_board_status(buf, cap);
    return (n > 0) ? (size_t)n : 0;
  }
  int n = snprintf(buf, cap, "CAN off (TWAI down)");
  return (n > 0) ? (size_t)n : 0;
}

static size_t main_line_flags(char *buf, size_t cap, void *ctx) {
  (void)ctx;
  return app_state_debug_flags_line_write(buf, cap);
}

static size_t main_line_imu(char *buf, size_t cap, void *ctx) {
  (void)ctx;
  return imu_status_line_write(buf, cap);
}

static size_t main_line_height(char *buf, size_t cap, void *ctx) {
  (void)ctx;
  return height_status_line_write(buf, cap);
}

static size_t main_line_servo(char *buf, size_t cap, void *ctx) {
  (void)ctx;
  return servo_drive_status_line_write(buf, cap);
}

static size_t main_line_rudder_demand(char *buf, size_t cap, void *ctx) {
  (void)ctx;
  uint16_t pot = 50;
  bool pot_fresh = app_state_pot_fresh(500, &pot);
  int n =
      snprintf(buf, cap, "Rudder: %s %u%%", pot_fresh ? "CAN" : "DEMO", (unsigned)pot);
  return (n > 0) ? (size_t)n : 0;
}

static const lvgl_status_line_t MAIN_STATUS_LINES[] = {
    {.write = main_line_title, .ctx = NULL},
    {.write = main_line_display, .ctx = NULL},
    {.write = main_line_can, .ctx = NULL},
    {.write = main_line_flags, .ctx = NULL},
    {.write = main_line_imu, .ctx = NULL},
    {.write = main_line_height, .ctx = NULL},
    {.write = main_line_servo, .ctx = NULL},
    {.write = main_line_rudder_demand, .ctx = NULL},
};

static void main_status_display_init(void) {
  app_state_set_display(false);
  if (tdisplays3_init(&s_tdisp_board) != ESP_OK) {
    ESP_LOGW(TAG, "tdisplays3_init failed — serial only");
    return;
  }
  if (s_tdisp_board.display == NULL) {
    ESP_LOGW(TAG, "Display handle null — serial only");
    return;
  }
  if (!tdisplays3_display_lock(200)) {
    ESP_LOGW(TAG, "LVGL display lock timeout — serial only");
    return;
  }
  app_state_set_display(true);

  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x101018), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

  lv_obj_t *lbl = lv_label_create(scr);
  if (lbl == NULL) {
    ESP_LOGW(TAG, "Label create failed — display online without debug text");
    tdisplays3_display_unlock();
    return;
  }
  lv_obj_set_style_text_color(lbl, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_text_font(lbl, LV_FONT_DEFAULT, LV_PART_MAIN);
  {
    const lv_coord_t hres = lv_display_get_horizontal_resolution(s_tdisp_board.display);
    lv_obj_set_width(lbl, hres > 8 ? hres - 8 : hres);
  }
  lv_label_set_long_mode(lbl, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_line_space(lbl, 2, LV_PART_MAIN);
  lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 4, 4);
  lv_label_set_text(lbl, "MAIN controller\nBooting…");
  tdisplays3_display_unlock();
  ESP_LOGI(TAG, "Display debug label ready");

  const size_t nlines = sizeof(MAIN_STATUS_LINES) / sizeof(MAIN_STATUS_LINES[0]);
  if (lvgl_status_display_start(&s_status_dsp, lbl, MAIN_STATUS_LINES, nlines, 200, 1024) != ESP_OK) {
    ESP_LOGW(TAG, "status display timer failed — static boot text only");
  }
}

static void task_loop(void *arg) {
  (void)arg;
  for (;;) {
    app_state_t st;
    app_state_get(&st);

    uint16_t pot_pct = 50;
    bool pot_from_can = app_state_pot_fresh(500, &pot_pct);
    if (!pot_from_can) {
      pot_pct = 50;
    }

    if (st.servo_ok && servo_drive_is_ready()) {
      servo_drive_set_pct(pot_pct);
      int16_t adeg, bdeg;
      servo_drive_get_commanded_deg(&adeg, &bdeg);
      uint8_t sp[4];
      memcpy(sp, &adeg, 2);
      memcpy(sp + 2, &bdeg, 2);
      if (can_is_ready()) {
        (void)can_tx(CAN_ID_SERVO_POS, sp, sizeof(sp));
      }
    }

    if (can_is_ready()) {
      if (st.imu_ok) {
        float pitch = 0, roll = 0, yaw = 0;
        if (imu_get_pitch_roll_yaw(&pitch, &roll, &yaw) == ESP_OK) {
          int16_t pi = (int16_t)lroundf(pitch);
          int16_t ri = (int16_t)lroundf(roll);
          int16_t yi = (int16_t)lroundf(yaw);
          uint8_t att[6];
          memcpy(att, &pi, 2);
          memcpy(att + 2, &ri, 2);
          memcpy(att + 4, &yi, 2);
          (void)can_tx(CAN_ID_ATTITUDE, att, sizeof(att));
        }
      }

      if (st.height_ok) {
        int32_t hcm = -1;
        if (height_get_cm(&hcm) == ESP_OK && hcm >= 0 && hcm <= (int32_t)UINT16_MAX) {
          uint16_t hu = (uint16_t)hcm;
          uint8_t hb[2];
          memcpy(hb, &hu, sizeof(hu));
          (void)can_tx(CAN_ID_HEIGHT, hb, sizeof(hb));
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(kTaskPeriodMs));
  }
}

void app_main(void) {
  app_state_init();
  main_status_display_init();

  const bool servo_ok = (servo_drive_init() == ESP_OK);
  app_state_set_servo(servo_ok);
  if (!servo_ok) {
    ESP_LOGW(TAG, "servo init failed");
  }

  if (xTaskCreate(task_loop, "io", 4096, NULL, kWorkTaskPrio, NULL) != pdPASS) {
    ESP_LOGE(TAG, "io task create failed");
  }

  const bool can_ok = (init_retry("CAN", can_start, 3) == ESP_OK);
  app_state_set_can(can_ok);
  if (!can_ok) {
    ESP_LOGW(TAG, "CAN off");
  }

  const bool imu_ok = (init_retry("IMU", imu_init, 3) == ESP_OK);
  app_state_set_imu(imu_ok);
  if (!imu_ok) {
    ESP_LOGW(TAG, "IMU off — no attitude CAN");
  }

  const bool height_ok = (init_retry("Height", height_init, 3) == ESP_OK);
  app_state_set_height(height_ok);
  if (!height_ok) {
    ESP_LOGW(TAG, "height sensor off — no height CAN");
  }
}
