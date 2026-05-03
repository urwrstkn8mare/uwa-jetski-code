#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "dashboard_ui.h"

void can_ui_bridge_init(void);

void can_ui_bridge_on_rx(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp);

void can_ui_bridge_merge_demo(dashboard_data_t *data, uint32_t demo_elapsed_ms);

bool can_ui_bridge_got_frame(void);
