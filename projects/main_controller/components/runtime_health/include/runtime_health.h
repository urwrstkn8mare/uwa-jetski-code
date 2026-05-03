#pragma once

#include <stdbool.h>

typedef struct {
  bool display_ok;
  bool servo_ok;
  bool can_ok;
  bool imu_ok;
  bool height_ok;
} runtime_health_t;

void runtime_health_set_display(bool ok);
void runtime_health_set_servo(bool ok);
void runtime_health_set_can(bool ok);
void runtime_health_set_imu(bool ok);
void runtime_health_set_height(bool ok);
void runtime_health_get(runtime_health_t *out);

/** Call once from app_main before any `runtime_health_set_*` (creates mutex). */
void runtime_health_early_init(void);
