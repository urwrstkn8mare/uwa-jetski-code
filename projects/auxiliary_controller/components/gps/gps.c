#include "gps.h"

#include <stdbool.h>

#include "can.h"
#include "can_ids.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "gps";

static SemaphoreHandle_t s_mux;
static int32_t s_lat_e7;
static int32_t s_lon_e7;
static int16_t s_speed_kmh_x10;
static int16_t s_heading_cdeg;
static uint8_t s_gps_fix;

static double nmea_coord_to_deg(const char *ddmm, char hemi) {
  if (ddmm == NULL || ddmm[0] == '\0') {
    return NAN;
  }
  double v = strtod(ddmm, NULL);
  int deg = (int)(v / 100.0);
  double minutes = fmod(v, 100.0);
  double dec = (double)deg + minutes / 60.0;
  if (hemi == 'S' || hemi == 'W') {
    dec = -dec;
  }
  return dec;
}

static bool parse_rmc_tokens(char **fields, int n, int32_t *lat_e7, int32_t *lon_e7, int16_t *sp_x10,
                             int16_t *hd_cdeg, uint8_t *fix) {
  if (n < 12) {
    return false;
  }
  const char *stat = fields[2];
  if (stat == NULL || stat[0] != 'A') {
    *fix = 0;
    return false;
  }
  *fix = 1;
  double latd = nmea_coord_to_deg(fields[3], fields[4] ? fields[4][0] : 'N');
  double lond = nmea_coord_to_deg(fields[5], fields[6] ? fields[6][0] : 'E');
  if (isnan(latd) || isnan(lond)) {
    return false;
  }
  *lat_e7 = (int32_t)llround(latd * 1e7);
  *lon_e7 = (int32_t)llround(lond * 1e7);

  double knots = strtod(fields[7] != NULL ? fields[7] : "0", NULL);
  double kmh = knots * 1.852;
  int ix = (int)llround(kmh * 10.0);
  if (ix > 32767) {
    ix = 32767;
  }
  if (ix < -32768) {
    ix = -32768;
  }
  *sp_x10 = (int16_t)ix;

  double course = strtod(fields[8] != NULL ? fields[8] : "0", NULL);
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
  *hd_cdeg = (int16_t)hc;
  return true;
}

static void handle_nmea_line(const char *line) {
  if (line == NULL || line[0] != '$') {
    return;
  }
  if (strstr(line, "RMC") == NULL) {
    return;
  }

  char buf[128];
  size_t n = strlcpy(buf, line, sizeof(buf));
  if (n >= sizeof(buf)) {
    return;
  }

  char *tok;
  char *save = NULL;
  char *fields[20];
  int fi = 0;
  for (tok = strtok_r(buf, ",", &save); tok != NULL && fi < 20; tok = strtok_r(NULL, ",", &save)) {
    fields[fi++] = tok;
  }
  int32_t la = 0, lo = 0;
  int16_t sp = 0, hd = 0;
  uint8_t fix = 0;
  if (!parse_rmc_tokens(fields, fi, &la, &lo, &sp, &hd, &fix)) {
    return;
  }

  uint8_t b8[8];
  memcpy(&b8[0], &la, sizeof(la));
  memcpy(&b8[4], &lo, sizeof(lo));
  can_tx(CAN_ID_GPS_POSITION, b8, sizeof(b8));

  memset(b8, 0, sizeof(b8));
  memcpy(&b8[0], &sp, sizeof(sp));
  memcpy(&b8[2], &hd, sizeof(hd));
  b8[4] = fix;
  can_tx(CAN_ID_GPS_VELOCITY, b8, 5);

  if (xSemaphoreTake(s_mux, pdMS_TO_TICKS(10)) == pdTRUE) {
    s_lat_e7 = la;
    s_lon_e7 = lo;
    s_speed_kmh_x10 = sp;
    s_heading_cdeg = hd;
    s_gps_fix = fix;
    xSemaphoreGive(s_mux);
  }
}

static void gps_uart_task(void *arg) {
  (void)arg;
  const uart_port_t port = (uart_port_t)CONFIG_GPS_UART_PORT_NUM;
  uart_config_t uc = {
      .baud_rate = CONFIG_GPS_UART_BAUD,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_ERROR_CHECK(uart_driver_install(port, 2048, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(port, &uc));
  ESP_ERROR_CHECK(uart_set_pin(port, CONFIG_GPS_UART_TX_GPIO, CONFIG_GPS_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  char line[160];
  size_t li = 0;
  for (;;) {
    uint8_t ch;
    int r = uart_read_bytes(port, &ch, 1, pdMS_TO_TICKS(500));
    if (r <= 0) {
      continue;
    }
    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      line[li] = '\0';
      if (li > 0) {
        handle_nmea_line(line);
      }
      li = 0;
      continue;
    }
    if (li + 1 < sizeof(line)) {
      line[li++] = (char)ch;
    } else {
      li = 0;
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
  xTaskCreate(gps_uart_task, "gps_uart", 4096, NULL, 5, NULL);
  ESP_LOGI(TAG, "Neo-6M task started (UART %d)", CONFIG_GPS_UART_PORT_NUM);
}

void gps_get_snapshot(int32_t *lat_e7, int32_t *lon_e7, int16_t *speed_kmh_x10, int16_t *heading_cdeg,
                      uint8_t *fix) {
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
    *fix = s_gps_fix;
  }
  xSemaphoreGive(s_mux);
}
