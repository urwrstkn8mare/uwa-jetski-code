#include "dashboard_can.h"

#include "can.h"
#include "can_ids.h"
#include "status_ui.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

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

/* Double-buffered CAN data to prevent race between RX task and paint functions.
 * The RX task writes to s_rx_data, then atomically swaps s_have_* flags.
 * Paint functions read under the display lock. */
typedef struct {
  int16_t pitch_deg;
  int16_t roll_deg;
  int16_t yaw_deg;
  int16_t height_cm;
  float servo_a_deg;
  float servo_b_deg;
  float speed_knots;
  float heading_deg;
  uint16_t pot_pct;
  int16_t height_target_cm;
  bool have_attitude;
  bool have_height;
  bool have_servo;
  bool have_gps_vel;
  bool have_pot;
  bool have_ctrl;
} can_rx_snapshot_t;

static can_rx_snapshot_t s_rx_data;
static dashboard_can_lock_fn_t s_lock;
static dashboard_can_unlock_fn_t s_unlock;
static void *s_lock_ctx;

static void paint_attitude(void) {
  if (!s_rx_data.have_attitude) {
    return;
  }
  int32_t heading = (int32_t)s_rx_data.yaw_deg % 360;
  if (heading < 0) {
    heading += 360;
  }
  dashboard_ui_set_attitude((int32_t)s_rx_data.roll_deg, (int32_t)s_rx_data.pitch_deg, heading);
}

static void paint_height(void) {
  if (!s_rx_data.have_height) {
    return;
  }
  int32_t target = s_rx_data.have_ctrl ? (int32_t)s_rx_data.height_target_cm : 25;
  dashboard_ui_set_height((int32_t)s_rx_data.height_cm, target);
}

static void paint_servo(void) {
  if (!s_rx_data.have_servo) {
    return;
  }
  dashboard_ui_set_elevons((int32_t)lroundf(s_rx_data.servo_a_deg),
                           (int32_t)lroundf(s_rx_data.servo_b_deg));
}

static void paint_gps_vel(void) {
  if (!s_rx_data.have_gps_vel) {
    return;
  }
  dashboard_ui_set_speed((int32_t)lroundf(s_rx_data.speed_knots * 1.852f));
  if (!s_rx_data.have_attitude) {
    int32_t h = (int32_t)lroundf(s_rx_data.heading_deg);
    h %= 360;
    if (h < 0) {
      h += 360;
    }
    dashboard_ui_set_attitude(0, 0, h);
  }
}

static void paint_pot(void) {
  if (!s_rx_data.have_pot) {
    return;
  }
  int32_t rudder = ((int32_t)s_rx_data.pot_pct * 40) / 100 - 20;
  dashboard_ui_set_rudder(rudder);
}

static void push_can_status(void) {
  char buf[96];
  int n = can_snprintf_board_status(buf, sizeof(buf));
  if (n > 0) {
    status_ui_update("CAN", "%s", buf);
  }
}

static void on_can_rx(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp) {
  (void)timestamp;
  if (buffer == NULL) {
    return;
  }
  switch (header_id) {
  case CAN_ID_ATTITUDE: {
    if (sizeof(can_attitude_t) <= 8) {
      can_attitude_t att;
      memcpy(&att, buffer, sizeof(att));
      s_rx_data.pitch_deg = att.pitch_deg;
      s_rx_data.roll_deg = att.roll_deg;
      s_rx_data.yaw_deg = att.yaw_deg;
      s_rx_data.have_attitude = true;
      with_lock(paint_attitude);
    }
    break;
  }
  case CAN_ID_HEIGHT: {
    can_height_t hb;
    memcpy(&hb, buffer, sizeof(hb));
    s_rx_data.height_cm = (int16_t)hb.height_cm;
    s_rx_data.have_height = true;
    with_lock(paint_height);
    break;
  }
  case CAN_ID_SERVO_POS: {
    can_servo_pos_t sp;
    memcpy(&sp, buffer, sizeof(sp));
    if (sp.channel == 0) {
      s_rx_data.servo_a_deg = sp.deg;
    } else {
      s_rx_data.servo_b_deg = sp.deg;
    }
    s_rx_data.have_servo = true;
    with_lock(paint_servo);
    break;
  }
  case CAN_ID_GPS_VELOCITY: {
    can_gps_velocity_t gv;
    memcpy(&gv, buffer, sizeof(gv));
    s_rx_data.speed_knots = gv.speed_knots;
    s_rx_data.heading_deg = gv.course_deg;
    s_rx_data.have_gps_vel = true;
    with_lock(paint_gps_vel);
    break;
  }
  case CAN_ID_POTENTIOMETER: {
    can_potentiometer_t pot;
    memcpy(&pot, buffer, sizeof(pot));
    s_rx_data.pot_pct = (pot.value > 100) ? 100 : pot.value;
    s_rx_data.have_pot = true;
    with_lock(paint_pot);
    break;
  }
  case CAN_ID_GPS_POSITION: {
    can_gps_position_t gp;
    memcpy(&gp, buffer, sizeof(gp));
    char gps_buf[48];
    snprintf(gps_buf, sizeof(gps_buf), "lat=%.4f lon=%.4f", (double)gp.lat_deg, (double)gp.lon_deg);
    status_ui_update("CAN_GPS", "%s", gps_buf);
    break;
  }
  case CAN_ID_CTRL_STATUS: {
    can_ctrl_status_t cs;
    memcpy(&cs, buffer, sizeof(cs));
    s_rx_data.height_target_cm = cs.height_target_cm;
    if (!s_rx_data.have_height) {
      s_rx_data.height_cm = cs.height_current_cm;
      s_rx_data.have_height = true;
    }
    s_rx_data.have_ctrl = true;
    with_lock(paint_height);
    break;
  }
  default:
    break;
  }
  push_can_status();
}

esp_err_t dashboard_can_attach(dashboard_can_lock_fn_t lock, dashboard_can_unlock_fn_t unlock,
                               void *lock_ctx) {
  if (lock == NULL || unlock == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  memset(&s_rx_data, 0, sizeof(s_rx_data));
  s_lock = lock;
  s_unlock = unlock;
  s_lock_ctx = lock_ctx;
  return ESP_OK;
}

esp_err_t dashboard_can_start(void) { return can_init(on_can_rx); }
