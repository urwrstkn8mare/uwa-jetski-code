#pragma once

#include <stdbool.h>
#include <stdint.h>

void rudder_on_can_rx(const uint8_t buffer[8], uint32_t header_id);

bool rudder_is_fresh(uint32_t max_age_ms, uint16_t *pct_out);
