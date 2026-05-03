/*
 * Auxiliary controller (T-Display-S3)
 *
 * - Rudder potentiometer → CAN (demand %) — detailed attitude/servos/GPS HUD live on main + dashboard projects.
 * - Neo-6M GPS → CAN position/velocity + local GPS status on this display.
 */

#include "aux_panel_ui.h"
#include "can.h"
#include "esp_log.h"
#include "gps.h"
#include "rudder_pot.h"

#include <stdint.h>

static const char *TAG = "aux_main";

void app_main(void) {
  aux_panel_ui_init();

  if (can_init(NULL) != ESP_OK) {
    ESP_LOGE(TAG, "CAN failed");
    return;
  }

  gps_init();
  if (rudder_pot_init() != ESP_OK) {
    ESP_LOGW(TAG, "Rudder ADC not initialised");
  }
}
