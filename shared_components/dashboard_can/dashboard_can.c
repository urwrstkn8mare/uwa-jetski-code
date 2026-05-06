#include "dashboard_can.h"

#include "can.h"
#include "can_ids.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

static dashboard_ui_t *s_ui;
static dashboard_can_lock_fn_t s_lock;
static dashboard_can_unlock_fn_t s_unlock;
static void *s_lock_ctx;

static void with_lock(void (*fn)(void)) {
  if (s_lock == NULL || s_unlock == NULL || fn == NULL) {
    return;
  }
  if (s_lock(-1, s_lock_ctx) == ESP_OK) {
    fn();
    s_unlock(s_lock_ctx);
  }
}

static int16_t s_pitch_deg;
static int16_t s_roll_deg;
static int16_t s_yaw_deg;
static int16_t s_height_cm;
static int16_t s_servo_a_deg;
static int16_t s_servo_b_deg;
static int16_t s_speed_kmh_x10;
static int16_t s_heading_cdeg;
static uint16_t s_pot_pct;

static bool s_have_attitude;
static bool s_have_height;
static bool s_have_servo;
static bool s_have_gps_vel;
static bool s_have_pot;

static void paint_attitude(void) {
  if (s_ui == NULL || !s_have_attitude) {
    return;
  }
  int32_t heading = (int32_t)s_yaw_deg % 360;
  if (heading < 0) {
    heading += 360;
  }
  dashboard_ui_set_attitude(s_ui, (int32_t)s_roll_deg, (int32_t)s_pitch_deg, heading);
}

static void paint_height(void) {
  if (s_ui == NULL || !s_have_height) {
    return;
  }
  dashboard_ui_set_height(s_ui, (int32_t)s_height_cm, 25);
}

static void paint_servo(void) {
  if (s_ui == NULL || !s_have_servo) {
    return;
  }
  dashboard_ui_set_elevons(s_ui, (int32_t)s_servo_a_deg, (int32_t)s_servo_b_deg);
}

static void paint_gps_vel(void) {
  if (s_ui == NULL || !s_have_gps_vel) {
    return;
  }
  dashboard_ui_set_speed(s_ui, (int32_t)(s_speed_kmh_x10 / 10));
  /* Use GPS heading only if attitude hasn't set it recently. */
  if (!s_have_attitude) {
    int32_t h = (int32_t)(s_heading_cdeg / 100);
    if (h < 0) {
      h += 360;
    }
    h %= 360;
    dashboard_ui_set_attitude(s_ui, 0, 0, h);
  }
}

static void paint_pot(void) {
  if (s_ui == NULL || !s_have_pot) {
    return;
  }
  int32_t rudder = ((int32_t)s_pot_pct * 40) / 100 - 20;
  dashboard_ui_set_rudder(s_ui, rudder);
}

static void on_can_rx(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp) {
  (void)timestamp;
  if (s_ui == NULL || buffer == NULL) {
    return;
  }
  switch (header_id) {
  case CAN_ID_ATTITUDE:
    if (sizeof(int16_t) * 3 <= 8) {
      memcpy(&s_pitch_deg, &buffer[0], sizeof(s_pitch_deg));
      memcpy(&s_roll_deg, &buffer[2], sizeof(s_roll_deg));
      memcpy(&s_yaw_deg, &buffer[4], sizeof(s_yaw_deg));
      s_have_attitude = true;
      with_lock(paint_attitude);
    }
    break;
  case CAN_ID_HEIGHT: {
    uint16_t h;
    memcpy(&h, &buffer[0], sizeof(h));
    s_height_cm = (int16_t)h;
    s_have_height = true;
    with_lock(paint_height);
    break;
  }
  case CAN_ID_SERVO_POS:
    memcpy(&s_servo_a_deg, &buffer[0], sizeof(s_servo_a_deg));
    memcpy(&s_servo_b_deg, &buffer[2], sizeof(s_servo_b_deg));
    s_have_servo = true;
    with_lock(paint_servo);
    break;
  case CAN_ID_GPS_VELOCITY:
    memcpy(&s_speed_kmh_x10, &buffer[0], sizeof(s_speed_kmh_x10));
    memcpy(&s_heading_cdeg, &buffer[2], sizeof(s_heading_cdeg));
    s_have_gps_vel = true;
    with_lock(paint_gps_vel);
    break;
  case CAN_ID_POTENTIOMETER: {
    uint16_t v;
    memcpy(&v, buffer, sizeof(v));
    s_pot_pct = (v > 100) ? 100 : v;
    s_have_pot = true;
    with_lock(paint_pot);
    break;
  }
  default:
    break;
  }
}

esp_err_t dashboard_can_attach(dashboard_ui_t *ui, dashboard_can_lock_fn_t lock, dashboard_can_unlock_fn_t unlock,
                               void *lock_ctx) {
  if (ui == NULL || lock == NULL || unlock == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  s_ui = ui;
  s_lock = lock;
  s_unlock = unlock;
  s_lock_ctx = lock_ctx;
  s_have_attitude = false;
  s_have_height = false;
  s_have_servo = false;
  s_have_gps_vel = false;
  s_have_pot = false;
  return ESP_OK;
}

esp_err_t dashboard_can_start(void) { return can_init(on_can_rx); }

size_t dashboard_can_status_strip_write(char *buffer, size_t len, void *user) {
  (void)user;
  if (buffer == NULL || len == 0) {
    return 0;
  }
  int written = can_snprintf_board_status(buffer, len);
  return (written > 0) ? (size_t)written : 0;
}
