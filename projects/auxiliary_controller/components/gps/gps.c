#include "gps.h"

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "can.h"
#include "can_ids.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "gpgga.h"
#include "gprmc.h"
#include "nmea.h"
#include "parser.h"

static const char *TAG = "gps";

static SemaphoreHandle_t s_mux;
static int32_t s_lat_e7;
static int32_t s_lon_e7;
static int16_t s_speed_kmh_x10;
static int16_t s_heading_cdeg;
static uint8_t s_fix_live;
static uint8_t s_gga_fix_qual;
static uint8_t s_sats_used;

static uint32_t s_bytes_rx_total;
static uint32_t s_nmea_frames;
static int64_t s_boot_us;

static uint32_t s_uart_lines_total;
static uint32_t s_nmea_parse_ok;
static uint32_t s_nmea_parse_fail;
static int64_t s_last_uart_line_us;

static char s_prev_uart_sentence[104];
static char s_last_uart_sentence[104];

static void gps_sanitize_line(char *dst, size_t dst_sz, const char *src) {
  if (dst_sz == 0) {
    return;
  }
  size_t j = 0;
  for (size_t i = 0; src[i] != '\0' && j + 1 < dst_sz; i++) {
    unsigned char c = (unsigned char)src[i];
    dst[j++] = (unsigned char)isprint((int)c) != 0 ? (char)c : '.';
  }
  dst[j] = '\0';
}

static void gps_on_complete_uart_line(const char *line_terminated) {
  if (line_terminated == NULL || line_terminated[0] == '\0') {
    return;
  }
  if (s_mux != NULL && xSemaphoreTake(s_mux, pdMS_TO_TICKS(10)) == pdTRUE) {
    s_uart_lines_total++;
    s_last_uart_line_us = esp_timer_get_time();
    strncpy(s_prev_uart_sentence, s_last_uart_sentence, sizeof(s_prev_uart_sentence) - 1);
    s_prev_uart_sentence[sizeof(s_prev_uart_sentence) - 1] = '\0';
    gps_sanitize_line(s_last_uart_sentence, sizeof(s_last_uart_sentence), line_terminated);
    xSemaphoreGive(s_mux);
  }
}

static double nmea_latitude_to_decimal(const nmea_position *p) {
  double v = (double)p->degrees + p->minutes / 60.0;
  if (p->cardinal == NMEA_CARDINAL_DIR_SOUTH) {
    v = -v;
  }
  return v;
}

static double nmea_longitude_to_decimal(const nmea_position *p) {
  double v = (double)p->degrees + p->minutes / 60.0;
  if (p->cardinal == NMEA_CARDINAL_DIR_WEST) {
    v = -v;
  }
  return v;
}

static void gps_tx_can_and_cache(int32_t la, int32_t lo, int16_t sp, int16_t hd, uint8_t fix_flag) {
  uint8_t b8[8];
  memcpy(&b8[0], &la, sizeof(la));
  memcpy(&b8[4], &lo, sizeof(lo));
  can_tx(CAN_ID_GPS_POSITION, b8, sizeof(b8));

  memset(b8, 0, sizeof(b8));
  memcpy(&b8[0], &sp, sizeof(sp));
  memcpy(&b8[2], &hd, sizeof(hd));
  b8[4] = fix_flag;
  can_tx(CAN_ID_GPS_VELOCITY, b8, 5);

  if (xSemaphoreTake(s_mux, pdMS_TO_TICKS(10)) != pdTRUE) {
    return;
  }
  s_lat_e7 = la;
  s_lon_e7 = lo;
  s_speed_kmh_x10 = sp;
  s_heading_cdeg = hd;
  s_fix_live = fix_flag;
  xSemaphoreGive(s_mux);
}

/**
 * igrr/libnmea expects CRLF-terminated lines and parses in place — use a writable buffer ≤ NMEA_MAX_LENGTH.
 */
static void append_crlf_line(char *dst, size_t dst_sz, const char *src, size_t *out_len_including_crlf) {
  size_t w = 0;
  for (size_t i = 0; src[i] != '\0' && w + 3 < dst_sz; i++) {
    if (src[i] != '\r') {
      dst[w++] = src[i];
    }
  }
  dst[w++] = '\r';
  dst[w++] = '\n';
  dst[w] = '\0';
  *out_len_including_crlf = w;
}

/**
 * Parses a single NMEA0183 logical line into a freshly allocated sentence struct (caller must nmea_free).
 * libnmea mutates wb; refill from @p line each attempt.
 */
static nmea_s *parse_nmea_line_with_lib(const char *line, bool line_has_checksum_field) {
  char wb[NMEA_MAX_LENGTH + 8];
  size_t ln = 0;
  append_crlf_line(wb, sizeof(wb), line, &ln);
  if (ln > NMEA_MAX_LENGTH) {
    return NULL;
  }
  nmea_s *data = nmea_parse(wb, ln, line_has_checksum_field ? 1 : 0);
  if (data != NULL) {
    return data;
  }
  if (!line_has_checksum_field) {
    return NULL;
  }
  append_crlf_line(wb, sizeof(wb), line, &ln);
  return nmea_parse(wb, ln, 0);
}

static bool apply_gga_sentence(nmea_gpgga_s *gga) {
  uint8_t qual = gga->position_fix;
  if (qual > 8) {
    qual = 8;
  }

  if (gga->position_fix == 0) {
    if (xSemaphoreTake(s_mux, pdMS_TO_TICKS(10)) != pdTRUE) {
      return true;
    }
    s_gga_fix_qual = (uint8_t)gga->position_fix;
    s_fix_live = 0;
    xSemaphoreGive(s_mux);
    return true;
  }

  double latd = nmea_latitude_to_decimal(&gga->latitude);
  double lond = nmea_longitude_to_decimal(&gga->longitude);
  if (isnan(latd) || isnan(lond)) {
    return false;
  }

  int16_t spd_storage = 0;
  int16_t hdg_storage = 0;
  if (xSemaphoreTake(s_mux, pdMS_TO_TICKS(10)) != pdTRUE) {
    return false;
  }
  spd_storage = s_speed_kmh_x10;
  hdg_storage = s_heading_cdeg;
  s_gga_fix_qual = qual;
  uint8_t sats = (uint8_t)(gga->n_satellites < 0 ? 0 : gga->n_satellites);
  s_sats_used = sats > 99 ? 99 : sats;
  xSemaphoreGive(s_mux);

  int32_t gla = (int32_t)llround(latd * 1e7);
  int32_t glo = (int32_t)llround(lond * 1e7);
  gps_tx_can_and_cache(gla, glo, spd_storage, hdg_storage, (uint8_t)1);
  return true;
}

static bool apply_rmc_sentence(nmea_gprmc_s *rmc) {
  if (!rmc->valid) {
    return true;
  }
  double latd = nmea_latitude_to_decimal(&rmc->latitude);
  double lond = nmea_longitude_to_decimal(&rmc->longitude);
  if (isnan(latd) || isnan(lond)) {
    return false;
  }
  double knots = rmc->gndspd_knots;
  double kmh = knots * 1.852;
  int sp_ix = (int)llround(kmh * 10.0);
  if (sp_ix > 32767) {
    sp_ix = 32767;
  }
  if (sp_ix < -32768) {
    sp_ix = -32768;
  }

  double course = rmc->track_deg;
  while (course < 0) {
    course += 360.0;
  }
  while (course >= 360.0) {
    course -= 360.0;
  }
  int hc = (int)llround(course * 100.0);
  if (hc > 36000) {
    hc = 36000;
  }

  int32_t la = (int32_t)llround(latd * 1e7);
  int32_t lo = (int32_t)llround(lond * 1e7);
  gps_tx_can_and_cache(la, lo, (int16_t)sp_ix, (int16_t)hc, (uint8_t)1);
  return true;
}

static void handle_one_nmea_line(const char *line_in_place_copy_src) {
  if (line_in_place_copy_src == NULL || line_in_place_copy_src[0] != '$') {
    return;
  }

  const bool line_has_chk = strchr(line_in_place_copy_src, '*') != NULL;
  nmea_s *data = parse_nmea_line_with_lib(line_in_place_copy_src, line_has_chk);

  if (data == NULL) {
    if (s_mux != NULL && xSemaphoreTake(s_mux, pdMS_TO_TICKS(10)) == pdTRUE) {
      s_nmea_parse_fail++;
      xSemaphoreGive(s_mux);
    }
    return;
  }

  s_nmea_frames++;
  if (s_mux != NULL && xSemaphoreTake(s_mux, pdMS_TO_TICKS(10)) == pdTRUE) {
    s_nmea_parse_ok++;
    xSemaphoreGive(s_mux);
  }

  switch (data->type) {
  case NMEA_GPGGA:
    (void)apply_gga_sentence((nmea_gpgga_s *)data);
    break;
  case NMEA_GPRMC:
    (void)apply_rmc_sentence((nmea_gprmc_s *)data);
    break;
  default:
    break;
  }

  nmea_free(data);
}

static void gps_uart_task(void *arg) {
  (void)arg;
  (void)nmea_load_parsers();

  const uart_port_t port = (uart_port_t)CONFIG_GPS_UART_PORT_NUM;

  uart_config_t uc = {
      .baud_rate = CONFIG_GPS_UART_BAUD,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  ESP_ERROR_CHECK(uart_driver_install(port, 4096, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(port, &uc));
  int tx_gpio = CONFIG_GPS_UART_TX_GPIO;
  if (tx_gpio < 0) {
    tx_gpio = UART_PIN_NO_CHANGE;
  }
  ESP_ERROR_CHECK(uart_set_pin(port, tx_gpio, CONFIG_GPS_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  uart_flush_input(port);

  char linebuf[176];
  size_t line_len = 0;
  s_boot_us = esp_timer_get_time();
  bool logged_sample = false;

  for (;;) {
    uint8_t chunk[384];
    int n = uart_read_bytes(port, chunk, sizeof(chunk), pdMS_TO_TICKS(120));
    if (n > 0) {
      s_bytes_rx_total += (uint32_t)n;
      if (!logged_sample && s_bytes_rx_total > 120) {
        logged_sample = true;
        if (CONFIG_GPS_UART_TX_GPIO < 0) {
          ESP_LOGI(TAG, "UART%d RX GPIO %d active at %u baud (TX NC)", (int)port,
                   CONFIG_GPS_UART_RX_GPIO, (unsigned)CONFIG_GPS_UART_BAUD);
        } else {
          ESP_LOGI(TAG, "UART%d RX GPIO %d TX GPIO %d at %u baud", (int)port, CONFIG_GPS_UART_RX_GPIO,
                   CONFIG_GPS_UART_TX_GPIO, (unsigned)CONFIG_GPS_UART_BAUD);
        }
      }
      for (int i = 0; i < n; i++) {
        char ch = (char)chunk[i];
        if (ch == '\r') {
          continue;
        }
        if (ch == '\n') {
          linebuf[line_len] = '\0';
          if (line_len > 0) {
            char scratch[sizeof(linebuf)];
            strlcpy(scratch, linebuf, sizeof(scratch));
            gps_on_complete_uart_line(scratch);
            handle_one_nmea_line(scratch);
          }
          line_len = 0;
          continue;
        }
        if ((size_t)line_len + 1 < sizeof(linebuf)) {
          linebuf[line_len++] = ch;
        } else {
          line_len = 0;
        }
      }
    }

    const int64_t now = esp_timer_get_time();
    if (now - s_boot_us > (int64_t)12 * 1000000LL && s_nmea_frames == 0 && s_bytes_rx_total == 0) {
      ESP_LOGI(TAG,
               "GPS UART silence @%u baud; try 38400 or swap MCU RX/Neo TX (GPIO RX=%d)",
               (unsigned)CONFIG_GPS_UART_BAUD, CONFIG_GPS_UART_RX_GPIO);
      s_boot_us = now + (int64_t)120 * 1000000LL;
    }
  }
}

void gps_init(void) {
  static bool started;
  if (started) {
    return;
  }
  started = true;
  if (s_mux == NULL) {
    s_mux = xSemaphoreCreateMutex();
  }
  xTaskCreate(gps_uart_task, "gps_uart", 8192, NULL, 5, NULL);
  ESP_LOGI(TAG, "GPS UART task port %d RX GPIO %d %u baud (TX %s)", CONFIG_GPS_UART_PORT_NUM,
           CONFIG_GPS_UART_RX_GPIO, (unsigned)CONFIG_GPS_UART_BAUD,
           (CONFIG_GPS_UART_TX_GPIO < 0) ? "NC" : "wired");
}

void gps_get_snapshot(int32_t *lat_e7, int32_t *lon_e7, int16_t *speed_kmh_x10, int16_t *heading_cdeg,
                      uint8_t *fix, uint8_t *gga_quality_out) {
  if (gga_quality_out != NULL) {
    *gga_quality_out = 0;
  }

  if (s_mux == NULL || xSemaphoreTake(s_mux, pdMS_TO_TICKS(10)) != pdTRUE) {
    return;
  }
  if (lat_e7) {
    *lat_e7 = s_lat_e7;
  }
  if (lon_e7) {
    *lon_e7 = s_lon_e7;
  }
  if (speed_kmh_x10) {
    *speed_kmh_x10 = s_speed_kmh_x10;
  }
  if (heading_cdeg) {
    *heading_cdeg = s_heading_cdeg;
  }
  if (fix) {
    uint8_t f = (s_fix_live || s_gga_fix_qual > 0) ? (uint8_t)1 : (uint8_t)0;
    *fix = f;
  }
  if (gga_quality_out != NULL) {
    *gga_quality_out = s_gga_fix_qual;
  }
  xSemaphoreGive(s_mux);
}

void gps_get_live_debug(gps_live_debug_t *out) {
  if (out == NULL) {
    return;
  }
  memset(out, 0, sizeof(*out));
  out->uart_baud = (uint32_t)CONFIG_GPS_UART_BAUD;
  if (s_mux == NULL || xSemaphoreTake(s_mux, pdMS_TO_TICKS(15)) != pdTRUE) {
    return;
  }
  out->uart_bytes_rx = s_bytes_rx_total;
  out->uart_lines_rx = s_uart_lines_total;
  out->nmea_parse_ok = s_nmea_parse_ok;
  out->nmea_parse_fail = s_nmea_parse_fail;
  out->sats_used_last_gga = s_sats_used;
  strlcpy(out->last_sentence, s_last_uart_sentence, sizeof(out->last_sentence));
  strlcpy(out->prev_sentence, s_prev_uart_sentence, sizeof(out->prev_sentence));
  int64_t now = esp_timer_get_time();
  if (s_last_uart_line_us > 0LL && now >= s_last_uart_line_us) {
    out->ms_since_last_uart_line = (uint32_t)((now - s_last_uart_line_us) / 1000LL);
  } else {
    out->ms_since_last_uart_line = 0xffffffffu;
  }
  xSemaphoreGive(s_mux);
}
