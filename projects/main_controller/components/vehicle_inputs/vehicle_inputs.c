#include "vehicle_inputs.h"

#include "can_ids.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static SemaphoreHandle_t s_mux;
static uint16_t s_pot_pct;
static int32_t s_lat_e7;
static int32_t s_lon_e7;
static int16_t s_speed_kmh_x10;
static int16_t s_heading_cdeg;
static bool s_have_gps;
static bool s_have_pot;
static TickType_t s_pot_last_tick;

void vehicle_inputs_init(void) { s_mux = xSemaphoreCreateMutex(); }

void vehicle_inputs_on_can_rx(const uint8_t buffer[8], uint32_t header_id) {
  if (buffer == NULL || s_mux == NULL) {
    return;
  }
  if (xSemaphoreTake(s_mux, pdMS_TO_TICKS(20)) != pdTRUE) {
    return;
  }
  switch (header_id) {
  case CAN_ID_POTENTIOMETER: {
    uint16_t v;
    memcpy(&v, buffer, sizeof(v));
    s_pot_pct = (v > 100) ? 100 : v;
    s_have_pot = true;
    s_pot_last_tick = xTaskGetTickCount();
    break;
  }
  case CAN_ID_GPS_POSITION:
    memcpy(&s_lat_e7, &buffer[0], sizeof(s_lat_e7));
    memcpy(&s_lon_e7, &buffer[4], sizeof(s_lon_e7));
    s_have_gps = true;
    break;
  case CAN_ID_GPS_VELOCITY:
    memcpy(&s_speed_kmh_x10, &buffer[0], sizeof(s_speed_kmh_x10));
    memcpy(&s_heading_cdeg, &buffer[2], sizeof(s_heading_cdeg));
    break;
  default:
    break;
  }
  xSemaphoreGive(s_mux);
}

uint16_t vehicle_inputs_get_pot_pct(void) {
  uint16_t v = 0;
  if (s_mux && xSemaphoreTake(s_mux, pdMS_TO_TICKS(20)) == pdTRUE) {
    v = s_pot_pct;
    xSemaphoreGive(s_mux);
  }
  return v;
}

bool vehicle_inputs_get_pot_fresh(uint32_t max_age_ms, uint16_t *pot_pct_out) {
  if (s_mux == NULL || xSemaphoreTake(s_mux, pdMS_TO_TICKS(20)) != pdTRUE) {
    return false;
  }
  const bool have = s_have_pot;
  const uint16_t pct = s_pot_pct;
  const TickType_t last = s_pot_last_tick;
  xSemaphoreGive(s_mux);

  if (!have) {
    return false;
  }
  TickType_t now = xTaskGetTickCount();
  TickType_t max_age_ticks = pdMS_TO_TICKS(max_age_ms);
  if ((now - last) > max_age_ticks) {
    return false;
  }
  if (pot_pct_out != NULL) {
    *pot_pct_out = pct;
  }
  return true;
}

bool vehicle_inputs_get_gps(int32_t *lat_e7, int32_t *lon_e7, int16_t *speed_kmh_x10, int16_t *heading_cdeg,
                            bool *have_pos_fix) {
  if (s_mux == NULL || xSemaphoreTake(s_mux, pdMS_TO_TICKS(10)) != pdTRUE) {
    return false;
  }
  if (lat_e7) {
    *lat_e7 = s_lat_e7;
  }
  if (lon_e7) {
    *lon_e7 = s_lon_e7;
  }
  if (speed_kmh_x10) {
    *speed_kmh_x10 = s_speed_kmh_x10;
  }
  if (heading_cdeg) {
    *heading_cdeg = s_heading_cdeg;
  }
  if (have_pos_fix) {
    *have_pos_fix = s_have_gps;
  }
  xSemaphoreGive(s_mux);
  return true;
}
