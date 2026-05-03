/*
 * Auxiliary controller (T-Display-S3)
 *
 * - Rudder potentiometer → CAN (demand %) — detailed attitude/servos/GPS HUD live on main + dashboard projects.
 * - Neo-6M GPS → CAN position/velocity + local GPS status on this display.
 */

#include "aux_panel_ui.h"
#include "can.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps.h"
#include "rudder_pot.h"

#include <stdint.h>

static const char *TAG = "aux_main";

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

static esp_err_t can_init_retry(void) { return can_init(NULL); }

void app_main(void) {
  aux_panel_ui_init();

  if (init_retry("CAN", can_init_retry, 3) != ESP_OK) {
    ESP_LOGW(TAG, "CAN init failed — display still active; TX disabled until fixed");
  }

  gps_init();
  if (rudder_pot_init() != ESP_OK) {
    ESP_LOGW(TAG, "Rudder ADC not initialised");
  }
}
