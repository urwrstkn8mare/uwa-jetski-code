#include "can_ui_bridge.h"

#include "can_ids.h"
#include "dashboard_demo.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static SemaphoreHandle_t s_mux;
static bool s_got_frame;

static int16_t s_pitch_deg;
static int16_t s_roll_deg;
static int16_t s_height_cm;
static int16_t s_servo_a_deg;
static int16_t s_servo_b_deg;
static int32_t s_lat_e7;
static int32_t s_lon_e7;
static int16_t s_speed_kmh_x10;
static int16_t s_heading_cdeg;

static bool s_have_attitude;
static bool s_have_height;
static bool s_have_servo;
static bool s_have_gps_pos;
static bool s_have_gps_vel;
static bool s_have_pot;
static uint16_t s_pot_pct;

void can_ui_bridge_init(void) {
  s_mux = xSemaphoreCreateMutex();
}

void can_ui_bridge_on_rx(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp) {
  (void)timestamp;
  if (buffer == NULL || s_mux == NULL) {
    return;
  }
  if (xSemaphoreTake(s_mux, portMAX_DELAY) != pdTRUE) {
    return;
  }
  s_got_frame = true;

  switch (header_id) {
  case CAN_ID_ATTITUDE:
    if (sizeof(int16_t) * 2 <= 8) {
      memcpy(&s_pitch_deg, &buffer[0], sizeof(s_pitch_deg));
      memcpy(&s_roll_deg, &buffer[2], sizeof(s_roll_deg));
      s_have_attitude = true;
    }
    break;
  case CAN_ID_HEIGHT: {
    uint16_t h;
    memcpy(&h, &buffer[0], sizeof(h));
    s_height_cm = (int16_t)h;
    s_have_height = true;
    break;
  }
  case CAN_ID_SERVO_POS:
    memcpy(&s_servo_a_deg, &buffer[0], sizeof(s_servo_a_deg));
    memcpy(&s_servo_b_deg, &buffer[2], sizeof(s_servo_b_deg));
    s_have_servo = true;
    break;
  case CAN_ID_GPS_POSITION:
    memcpy(&s_lat_e7, &buffer[0], sizeof(s_lat_e7));
    memcpy(&s_lon_e7, &buffer[4], sizeof(s_lon_e7));
    s_have_gps_pos = true;
    break;
  case CAN_ID_GPS_VELOCITY:
    memcpy(&s_speed_kmh_x10, &buffer[0], sizeof(s_speed_kmh_x10));
    memcpy(&s_heading_cdeg, &buffer[2], sizeof(s_heading_cdeg));
    s_have_gps_vel = true;
    break;
  case CAN_ID_POTENTIOMETER: {
    uint16_t v;
    memcpy(&v, buffer, sizeof(v));
    s_pot_pct = (v > 100) ? 100 : v;
    s_have_pot = true;
    break;
  }
  default:
    break;
  }
  xSemaphoreGive(s_mux);
}

void can_ui_bridge_merge_demo(dashboard_data_t *data, uint32_t demo_elapsed_ms) {
  if (data == NULL) {
    return;
  }
  dashboard_demo_fill(data, demo_elapsed_ms);
  if (s_mux == NULL || xSemaphoreTake(s_mux, pdMS_TO_TICKS(50)) != pdTRUE) {
    return;
  }
  if (s_have_attitude) {
    data->pitch_deg = s_pitch_deg;
    data->roll_deg = s_roll_deg;
  }
  if (s_have_height) {
    data->height_cm = s_height_cm;
  }
  /* Rudder is the operator POT (CAN 0x102); not derived from actuator telemetry. */
  if (s_have_pot) {
    data->rudder_deg = ((int32_t)s_pot_pct * 40) / 100 - 20;
  }
  if (s_have_servo) {
    data->elevon_left_deg = s_servo_a_deg;
    data->elevon_right_deg = s_servo_b_deg;
  }
  if (s_have_gps_vel) {
    data->speed_kmh = s_speed_kmh_x10 / 10;
    int32_t h = s_heading_cdeg / 100;
    if (h < 0) {
      h += 360;
    }
    if (h >= 360) {
      h %= 360;
    }
    data->heading_deg = h;
  }
  (void)s_lat_e7;
  (void)s_lon_e7;
  (void)s_have_gps_pos;
  xSemaphoreGive(s_mux);
}

bool can_ui_bridge_got_frame(void) { return s_got_frame; }
