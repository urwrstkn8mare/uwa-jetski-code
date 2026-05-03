/*
 * Main controller (T-Display-S3)
 *
 * - IMU attitude + ultrasonic height → CAN (separate broadcast tasks).
 * - Rudder *position* is the POT % from the auxiliary controller (CAN); sent to direct PWM outputs (GPIO1/GPIO2).
 * - GPS → CAN snapshots shown on-panel via vehicle_inputs when CAN is up.
 *
 * CAN / IMU / height may be unplugged — init failures are soft; onboard UI shows status.
 */

#include "can.h"
#include "can_ids.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "height.h"
#include "imu.h"
#include "main_panel_ui.h"
#include "runtime_health.h"
#include "servo_drive.h"
#include "vehicle_inputs.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>

static const char *TAG = "main";

static void can_rx_dispatch(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp) {
  (void)timestamp;
  vehicle_inputs_on_can_rx(buffer, header_id);
}

static esp_err_t can_init_retry(void) { return can_init(can_rx_dispatch); }

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

/** ~20 Hz: pitch/roll → CAN_ID_ATTITUDE */
static void task_attitude_broadcast(void *arg) {
  (void)arg;
  for (;;) {
    float pitch, roll;
    if (!can_is_ready()) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    if (imu_get_pitch_roll(&pitch, &roll) == ESP_OK) {
      int16_t pi = (int16_t)lroundf(pitch);
      int16_t ri = (int16_t)lroundf(roll);
      uint8_t b[4];
      memcpy(&b[0], &pi, sizeof(pi));
      memcpy(&b[2], &ri, sizeof(ri));
      (void)can_tx(CAN_ID_ATTITUDE, b, sizeof(b));
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/** ~20 Hz: height → CAN_ID_HEIGHT */
static void task_height_broadcast(void *arg) {
  (void)arg;
  for (;;) {
    if (!can_is_ready()) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    int32_t hcm;
    if (height_get_cm(&hcm) == ESP_OK) {
      uint16_t hu = (uint16_t)hcm;
      uint8_t b[2];
      memcpy(b, &hu, sizeof(hu));
      (void)can_tx(CAN_ID_HEIGHT, b, sizeof(b));
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/**
 * ~20 Hz: POT % (rudder position demand) → PWM servos + CAN_ID_SERVO_POS carrying elevon/commanded
 * surface angles echoed from outputs (same mapping on A/B today).
 */
static void task_rudder_outputs(void *arg) {
  (void)arg;
  uint32_t loop = 0;
  for (;;) {
    uint16_t pot_cmd = 50;
    bool pot_fresh = vehicle_inputs_get_pot_fresh(500, &pot_cmd);
    servo_drive_set_pct(pot_cmd);

    int16_t a, b;
    servo_drive_get_commanded_deg(&a, &b);
    uint8_t sb[4];
    memcpy(&sb[0], &a, sizeof(a));
    memcpy(&sb[2], &b, sizeof(b));
    if (can_is_ready()) {
      (void)can_tx(CAN_ID_SERVO_POS, sb, sizeof(sb));
    }

    loop++;
    if (loop % 11 == 0 && servo_drive_is_ready()) {
      uint32_t pu0 = 0, pu1 = 0;
      servo_drive_get_pulse_us(&pu0, &pu1);
      uint16_t fb0 = 0, fb1 = 0;
      bool has_fb0 = servo_drive_get_feedback_us(0, &fb0);
      bool has_fb1 = servo_drive_get_feedback_us(1, &fb1);
      ESP_LOGI(TAG,
               "Servo PWM POT[%s] %upct cmd ch0 %" PRIu32 "µs ch1 %" PRIu32 "µs fb ch0 %s%u ch1 %s%u",
               pot_fresh ? "CAN" : "STALE", (unsigned)pot_cmd, pu0, pu1, has_fb0 ? "" : "NA/",
               (unsigned)fb0, has_fb1 ? "" : "NA/", (unsigned)fb1);
    }
    if (loop % 200 == 0 && can_is_ready()) {
      uint32_t tries, failures;
      can_get_tx_stats(&tries, &failures);
      if (failures > 0) {
        ESP_LOGW(TAG, "CAN TX %" PRIu32 " tries %" PRIu32 " fail", tries, failures);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void app_main(void) {
  runtime_health_early_init();
  vehicle_inputs_init();
  main_panel_ui_init();

  /* PWM rudder loop always runs; when CAN POT is missing demo mode keeps outputs active. */
  const bool servo_ok = (servo_drive_init() == ESP_OK);
  runtime_health_set_servo(servo_ok);
  if (!servo_ok) {
    ESP_LOGW(TAG, "Servo PWM init failed");
  }
  xTaskCreate(task_rudder_outputs, "rudder_out", 4096, NULL, 6, NULL);

  const bool can_ok = (init_retry("CAN", can_init_retry, 3) == ESP_OK);
  runtime_health_set_can(can_ok);
  if (!can_ok) {
    ESP_LOGW(TAG, "CAN not initialised — PWM/UI still run; no CAN RX/TX");
  }

  /* IMU/height startup can block; flags drive the onboard status line only. */
  const bool imu_ok = (init_retry("IMU", imu_init, 3) == ESP_OK);
  runtime_health_set_imu(imu_ok);
  if (!imu_ok) {
    ESP_LOGW(TAG, "IMU not available — attitude CAN frames skipped");
  }

  const bool height_ok = (init_retry("Height", height_init, 3) == ESP_OK);
  runtime_health_set_height(height_ok);
  if (!height_ok) {
    ESP_LOGW(TAG, "Height sensor not available — height CAN frames skipped");
  }

  if (imu_ok) {
    xTaskCreate(task_attitude_broadcast, "imu_tx", 4096, NULL, 6, NULL);
  }
  if (height_ok) {
    xTaskCreate(task_height_broadcast, "hgt_tx", 4096, NULL, 6, NULL);
  }
}
