#include "runtime_health.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static SemaphoreHandle_t s_mux;
static bool s_disp_ok;
static bool s_serv_ok;
static bool s_can_ok;
static bool s_imu_ok;
static bool s_h_ok;

static void run_set(bool *slot, bool v) {
  if (slot == NULL) {
    return;
  }
  if (s_mux != NULL && xSemaphoreTake(s_mux, pdMS_TO_TICKS(20)) == pdTRUE) {
    *slot = v;
    xSemaphoreGive(s_mux);
  }
}

void runtime_health_set_display(bool ok) { run_set(&s_disp_ok, ok); }

void runtime_health_set_servo(bool ok) { run_set(&s_serv_ok, ok); }

void runtime_health_set_can(bool ok) { run_set(&s_can_ok, ok); }

void runtime_health_set_imu(bool ok) { run_set(&s_imu_ok, ok); }

void runtime_health_set_height(bool ok) { run_set(&s_h_ok, ok); }

void runtime_health_get(runtime_health_t *out) {
  if (out == NULL) {
    return;
  }
  memset(out, 0, sizeof(*out));
  if (s_mux == NULL || xSemaphoreTake(s_mux, pdMS_TO_TICKS(20)) != pdTRUE) {
    return;
  }
  out->display_ok = s_disp_ok;
  out->servo_ok = s_serv_ok;
  out->can_ok = s_can_ok;
  out->imu_ok = s_imu_ok;
  out->height_ok = s_h_ok;
  xSemaphoreGive(s_mux);
}

void runtime_health_early_init(void) {
  if (s_mux != NULL) {
    return;
  }
  s_mux = xSemaphoreCreateMutex();
}
