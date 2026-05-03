#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * Rudder request + GPS snapshots from the auxiliary controller (via CAN).
 */
void vehicle_inputs_init(void);

void vehicle_inputs_on_can_rx(const uint8_t buffer[8], uint32_t header_id);

uint16_t vehicle_inputs_get_pot_pct(void);
bool vehicle_inputs_get_pot_fresh(uint32_t max_age_ms, uint16_t *pot_pct_out);

bool vehicle_inputs_get_gps(int32_t *lat_e7, int32_t *lon_e7, int16_t *speed_kmh_x10, int16_t *heading_cdeg,
                            bool *have_pos_fix);
