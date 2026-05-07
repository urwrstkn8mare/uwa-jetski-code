#include "app_state.h"
#include "can.h"
#include "can_ids.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "height.h"
#include "imu.h"
#include "status_ui.h"
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
static status_ui_t s_status_ui;

static void on_can_rx(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp) {
  (void)timestamp;
  app_state_on_can_rx(buffer, header_id);
}

static void main_status_lock(void) { tdisplays3_display_lock(200); }

static void main_status_unlock(void) { tdisplays3_display_unlock(); }

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

  const status_ui_cfg_t cfg = {
      .parent = scr,
      .lock_cb = main_status_lock,
      .unlock_cb = main_status_unlock,
      .min_interval_ms = 200,
  };
  if (status_ui_start(&s_status_ui, &cfg) != ESP_OK) {
    ESP_LOGW(TAG, "status display init failed");
  }

  tdisplays3_display_unlock();
  ESP_LOGI(TAG, "Display debug label ready");
}

static void task_loop(void *arg) {
  status_ui_t *disp = (status_ui_t *)arg;
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

      {
        char buf[128];
        size_t n = app_state_debug_flags_line_write(buf, sizeof(buf));
        if (n > 0) {
          status_ui_update(disp, "Flags", "%s", buf);
        }
      }

      uint16_t pot = 50;
      bool pot_fresh = app_state_pot_fresh(500, &pot);
      status_ui_update(disp, "Rudder",
                                 "Rudder: %s %u%%",
                                 pot_fresh ? "CAN" : "DEMO", (unsigned)pot);
    }

    vTaskDelay(pdMS_TO_TICKS(kTaskPeriodMs));
  }
}

void app_main(void) {
  app_state_init();
  main_status_display_init();

  const bool servo_ok = (servo_drive_init(status_ui_update, &s_status_ui) == ESP_OK);
  app_state_set_servo(servo_ok);
  if (!servo_ok) {
    ESP_LOGW(TAG, "servo init failed");
  }

  if (xTaskCreate(task_loop, "io", 4096, &s_status_ui, kWorkTaskPrio, NULL) != pdPASS) {
    ESP_LOGE(TAG, "io task create failed");
  }

  const bool can_ok = (can_init(on_can_rx) == ESP_OK);
  app_state_set_can(can_ok);
  if (!can_ok) {
    ESP_LOGW(TAG, "CAN off");
  }

  const bool imu_ok = (imu_init(status_ui_update, &s_status_ui) == ESP_OK);
  app_state_set_imu(imu_ok);
  if (!imu_ok) {
    ESP_LOGW(TAG, "IMU off — no attitude CAN");
  }

  const bool height_ok = (height_init(status_ui_update, &s_status_ui) == ESP_OK);
  app_state_set_height(height_ok);
  if (!height_ok) {
    ESP_LOGW(TAG, "height sensor off — no height CAN");
  }
}
