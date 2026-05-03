#pragma once

#include <stdint.h>

/**
 * Neo-6M NMEA UART reader: parses RMC, publishes CAN position/velocity, keeps latest fix for UI.
 */
void gps_init(void);

void gps_get_snapshot(int32_t *lat_e7, int32_t *lon_e7, int16_t *speed_kmh_x10, int16_t *heading_cdeg,
                      uint8_t *fix);
