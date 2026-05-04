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
#include "main_panel_ui.h"
#include "servo_drive.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>

static const char *TAG = "main";

enum { kTaskPeriodMs = 50 };
/* Match LVGL task priority in tdisplays3 (2) so this loop does not preempt LCD DMA. */
enum { kWorkTaskPrio = 2 };

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
        float pitch = 0, roll = 0;
        if (imu_get_pitch_roll(&pitch, &roll) == ESP_OK) {
          int16_t pi = (int16_t)lroundf(pitch);
          int16_t ri = (int16_t)lroundf(roll);
          uint8_t att[4];
          memcpy(att, &pi, 2);
          memcpy(att + 2, &ri, 2);
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
  main_panel_ui_init();

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
