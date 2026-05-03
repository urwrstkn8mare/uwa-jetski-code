/*
 * Main controller (T-Display-S3)
 *
 * - IMU attitude + ultrasonic height → CAN (separate broadcast tasks).
 * - Rudder *position* is the POT % from the auxiliary controller (CAN); mirrored on two PWM outputs.
 * - GPS → CAN snapshots shown on-panel via vehicle_inputs (separate node).
 */

#include "can.h"
#include "can_ids.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "height.h"
#include "imu.h"
#include "main_panel_ui.h"
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
    if (imu_get_pitch_roll(&pitch, &roll) == ESP_OK) {
      int16_t pi = (int16_t)lroundf(pitch);
      int16_t ri = (int16_t)lroundf(roll);
      uint8_t b[4];
      memcpy(&b[0], &pi, sizeof(pi));
      memcpy(&b[2], &ri, sizeof(ri));
      can_tx(CAN_ID_ATTITUDE, b, sizeof(b));
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/** ~20 Hz: height → CAN_ID_HEIGHT */
static void task_height_broadcast(void *arg) {
  (void)arg;
  for (;;) {
    int32_t hcm;
    if (height_get_cm(&hcm) == ESP_OK) {
      uint16_t hu = (uint16_t)hcm;
      uint8_t b[2];
      memcpy(b, &hu, sizeof(hu));
      can_tx(CAN_ID_HEIGHT, b, sizeof(b));
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/**
 * ~20 Hz: POT % (rudder position demand) → dual PWM + CAN_ID_SERVO_POS carrying elevon/commanded
 * surface angles echoed from outputs (same mapping on A/B today).
 */
static void task_rudder_outputs(void *arg) {
  (void)arg;
  uint32_t loop = 0;
  for (;;) {
    servo_drive_set_pct(vehicle_inputs_get_pot_pct());

    int16_t a, b;
    servo_drive_get_commanded_deg(&a, &b);
    uint8_t sb[4];
    memcpy(&sb[0], &a, sizeof(a));
    memcpy(&sb[2], &b, sizeof(b));
    can_tx(CAN_ID_SERVO_POS, sb, sizeof(sb));

    loop++;
    if (loop % 200 == 0) {
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
  vehicle_inputs_init();
  main_panel_ui_init();

  if (init_retry("IMU", imu_init, 3) != ESP_OK) {
    ESP_LOGW(TAG, "IMU not available");
  }
  if (init_retry("Height", height_init, 3) != ESP_OK) {
    ESP_LOGW(TAG, "Height sensor not available");
  }
  if (init_retry("CAN", can_init_retry, 3) != ESP_OK) {
    ESP_LOGE(TAG, "CAN required");
    return;
  }
  if (servo_drive_init() != ESP_OK) {
    ESP_LOGE(TAG, "LEDC rudder-output init failed");
    return;
  }

  xTaskCreate(task_attitude_broadcast, "imu_tx", 4096, NULL, 6, NULL);
  xTaskCreate(task_height_broadcast, "hgt_tx", 4096, NULL, 6, NULL);
  xTaskCreate(task_rudder_outputs, "rudder_out", 4096, NULL, 6, NULL);
}
