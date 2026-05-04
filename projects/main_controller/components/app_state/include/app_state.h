#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  bool display_ok;
  bool servo_ok;
  bool can_ok;
  bool imu_ok;
  bool height_ok;
} app_state_t;

void app_state_init(void);
void app_state_set_display(bool ok);
void app_state_set_servo(bool ok);
void app_state_set_can(bool ok);
void app_state_set_imu(bool ok);
void app_state_set_height(bool ok);
void app_state_get(app_state_t *out);

/** Rudder demand from auxiliary controller (CAN 0x102). */
void app_state_on_can_rx(const uint8_t buffer[8], uint32_t header_id);
/** True if POT was received within max_age_ms; writes 0–100 into *pot_pct_out. */
bool app_state_pot_fresh(uint32_t max_age_ms, uint16_t *pot_pct_out);
