#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "dashboard_ui.h"

typedef struct {
  bool have_height;
  bool have_servo;
  bool have_pot;
  bool have_attitude;
  int16_t height_cm;
  int16_t servo_a_deg;
  int16_t servo_b_deg;
  int16_t pitch_deg;
  int16_t roll_deg;
  int16_t yaw_deg;
  uint16_t pot_pct;
} can_ui_bridge_debug_t;

void can_ui_bridge_init(void);

void can_ui_bridge_on_rx(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp);

void can_ui_bridge_merge_demo(dashboard_data_t *data, uint32_t demo_elapsed_ms);

bool can_ui_bridge_got_frame(void);

void can_ui_bridge_get_debug(can_ui_bridge_debug_t *out);

size_t can_ui_bridge_snprintf_status(char *buffer, size_t len);
