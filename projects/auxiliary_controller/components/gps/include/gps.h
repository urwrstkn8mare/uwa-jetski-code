#pragma once

#include <stddef.h>
#include <stdint.h>

/**
 * Neo-6M NMEA UART reader: parses GGA + GPRMC/GNRMC (GGA fills lat/lon when fix_quality>0),
 * publishes CAN position/velocity, keeps latest snapshot for the panel.
 */
void gps_init(void);

void gps_get_snapshot(int32_t *lat_e7, int32_t *lon_e7, int16_t *speed_kmh_x10, int16_t *heading_cdeg,
                      uint8_t *have_position_fix, uint8_t *gga_fix_quality_out_or_null);

typedef struct {
  uint32_t uart_bytes_rx;
  uint32_t uart_lines_rx;
  uint32_t nmea_parse_ok;
  uint32_t nmea_parse_fail;
  uint32_t uart_baud;
  uint32_t ms_since_last_uart_line;
  uint8_t sats_used_last_gga;
  char last_sentence[104];
  char prev_sentence[104];
} gps_live_debug_t;

/** Thread-safe; fills @p out (cleared if mutex unavailable). */
void gps_get_live_debug(gps_live_debug_t *out);
