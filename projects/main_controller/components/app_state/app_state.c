#include "app_state.h"

#include "can_ids.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>

static SemaphoreHandle_t s_mx;

static bool s_disp_ok;
static bool s_serv_ok;
static bool s_can_ok;
static bool s_imu_ok;
static bool s_height_ok;

static uint16_t s_pot_pct = 50;
static bool s_have_pot;
static TickType_t s_pot_tick;

static void mtx_take(void) {
  if (s_mx != NULL) {
    (void)xSemaphoreTake(s_mx, portMAX_DELAY);
  }
}

static void mtx_give(void) {
  if (s_mx != NULL) {
    (void)xSemaphoreGive(s_mx);
  }
}

void app_state_init(void) {
  if (s_mx == NULL) {
    s_mx = xSemaphoreCreateMutex();
  }
}

void app_state_set_display(bool ok) {
  mtx_take();
  s_disp_ok = ok;
  mtx_give();
}
void app_state_set_servo(bool ok) {
  mtx_take();
  s_serv_ok = ok;
  mtx_give();
}
void app_state_set_can(bool ok) {
  mtx_take();
  s_can_ok = ok;
  mtx_give();
}
void app_state_set_imu(bool ok) {
  mtx_take();
  s_imu_ok = ok;
  mtx_give();
}
void app_state_set_height(bool ok) {
  mtx_take();
  s_height_ok = ok;
  mtx_give();
}

void app_state_get(app_state_t *out) {
  if (out == NULL) {
    return;
  }
  memset(out, 0, sizeof(*out));
  mtx_take();
  out->display_ok = s_disp_ok;
  out->servo_ok = s_serv_ok;
  out->can_ok = s_can_ok;
  out->imu_ok = s_imu_ok;
  out->height_ok = s_height_ok;
  mtx_give();
}

void app_state_on_can_rx(const uint8_t buffer[8], uint32_t header_id) {
  if (buffer == NULL || header_id != CAN_ID_POTENTIOMETER) {
    return;
  }

  mtx_take();
  uint16_t v;
  memcpy(&v, buffer, sizeof(v));
  s_pot_pct = (v > 100u) ? 100u : v;
  s_have_pot = true;
  s_pot_tick = xTaskGetTickCount();
  mtx_give();
}

size_t app_state_debug_flags_line_write(char *buf, size_t cap) {
  if (buf == NULL || cap == 0) {
    return 0;
  }
  app_state_t s;
  app_state_get(&s);
  int n =
      snprintf(buf, cap, "IMU:%s Height:%s Servo:%s", s.imu_ok ? "ok" : "off",
               s.height_ok ? "ok" : "off", (s.servo_ok) ? "ok" : "off");
  return (n > 0) ? (size_t)n : 0;
}

bool app_state_pot_fresh(uint32_t max_age_ms, uint16_t *pot_pct_out) {
  mtx_take();
  const bool have = s_have_pot;
  const uint16_t pct = s_pot_pct;
  const TickType_t last = s_pot_tick;
  mtx_give();

  if (!have) {
    return false;
  }
  const TickType_t max_ticks = pdMS_TO_TICKS(max_age_ms ? max_age_ms : 1);
  if ((xTaskGetTickCount() - last) > max_ticks) {
    return false;
  }
  if (pot_pct_out != NULL) {
    *pot_pct_out = pct;
  }
  return true;
}
