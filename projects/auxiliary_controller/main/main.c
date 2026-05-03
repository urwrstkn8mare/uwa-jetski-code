#include "can.h"
#include "can_ids.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "t_display_s3.h"
#include <math.h>
#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "aux_ctrl";

static SemaphoreHandle_t s_mux;
static int16_t s_rx_pitch;
static int16_t s_rx_roll;
static int16_t s_rx_height_cm;
static int16_t s_rx_servo_a;
static int16_t s_rx_servo_b;

static int32_t s_lat_e7;
static int32_t s_lon_e7;
static int16_t s_speed_kmh_x10;
static int16_t s_heading_cdeg;
static uint8_t s_gps_fix;

static adc_oneshot_unit_handle_t s_adc;

static tdisplays3_handle_t s_board;
static lv_obj_t *s_lbl;

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

static void pot_tx_task(void *arg) {
  (void)arg;
  for (;;) {
    if (s_adc != NULL) {
      int raw = 0;
      adc_channel_t ch;
      adc_unit_t u;
      if (adc_oneshot_io_to_channel(CONFIG_POT_GPIO_NUM, &u, &ch) == ESP_OK) {
        if (adc_oneshot_read(s_adc, ch, &raw) == ESP_OK) {
          int mn = CONFIG_POT_ADC_RAW_MIN;
          int mx = CONFIG_POT_ADC_RAW_MAX;
          int v = raw;
          if (v < mn) {
            v = mn;
          }
          if (v > mx) {
            v = mx;
          }
          uint16_t pct = 0;
          if (mx > mn) {
            pct = (uint16_t)(((int64_t)(v - mn) * 100) / (mx - mn));
          }
          uint8_t b[2];
          memcpy(b, &pct, sizeof(pct));
          can_tx(CAN_ID_POTENTIOMETER, b, sizeof(b));
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

static void can_rx_cb(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp) {
  (void)timestamp;
  if (buffer == NULL || s_mux == NULL) {
    return;
  }
  if (xSemaphoreTake(s_mux, pdMS_TO_TICKS(10)) != pdTRUE) {
    return;
  }
  switch (header_id) {
  case CAN_ID_ATTITUDE:
    memcpy(&s_rx_pitch, &buffer[0], sizeof(s_rx_pitch));
    memcpy(&s_rx_roll, &buffer[2], sizeof(s_rx_roll));
    break;
  case CAN_ID_HEIGHT: {
    uint16_t hu;
    memcpy(&hu, &buffer[0], sizeof(hu));
    s_rx_height_cm = (int16_t)hu;
    break;
  }
  case CAN_ID_SERVO_POS:
    memcpy(&s_rx_servo_a, &buffer[0], sizeof(s_rx_servo_a));
    memcpy(&s_rx_servo_b, &buffer[2], sizeof(s_rx_servo_b));
    break;
  default:
    break;
  }
  xSemaphoreGive(s_mux);
}

static void ui_timer(lv_timer_t *t) {
  (void)t;
  int16_t p = 0, r = 0, h = 0, sa = 0, sb = 0;
  int32_t la = 0, lo = 0;
  int16_t sp = 0, hd = 0;
  uint8_t fix = 0;
  if (s_mux && xSemaphoreTake(s_mux, pdMS_TO_TICKS(10)) == pdTRUE) {
    p = s_rx_pitch;
    r = s_rx_roll;
    h = s_rx_height_cm;
    sa = s_rx_servo_a;
    sb = s_rx_servo_b;
    la = s_lat_e7;
    lo = s_lon_e7;
    sp = s_speed_kmh_x10;
    hd = s_heading_cdeg;
    fix = s_gps_fix;
    xSemaphoreGive(s_mux);
  }
  uint32_t ca = 0, cf = 0;
  can_get_tx_stats(&ca, &cf);

  char buf[512];
  snprintf(buf, sizeof(buf),
           "Auxiliary controller\n"
           "CAN rx: P:%d R:%d H:%dcm\n"
           "Srv A:%d B:%d\n"
           "GPS fix:%u lat_e7:%ld lon_e7:%ld\n"
           "spd:%d.%d km/h hdg:%d.%02d\n"
           "CAN tx try:%" PRIu32 " fail:%" PRIu32 "",
           (int)p, (int)r, (int)h, (int)sa, (int)sb, (unsigned)fix, (long)la, (long)lo, sp / 10,
           abs(sp % 10), hd / 100, abs(hd % 100), ca, cf);

  if (tdisplays3_display_lock(120)) {
    lv_label_set_text(s_lbl, buf);
    tdisplays3_display_unlock();
  }
}

void app_main(void) {
  s_mux = xSemaphoreCreateMutex();

  ESP_ERROR_CHECK(tdisplays3_init(&s_board));
  if (!tdisplays3_display_lock(200)) {
    return;
  }
  s_lbl = lv_label_create(lv_screen_active());
  lv_obj_set_width(s_lbl, lv_display_get_horizontal_resolution(NULL) - 6);
  lv_label_set_long_mode(s_lbl, LV_LABEL_LONG_WRAP);
  lv_obj_align(s_lbl, LV_ALIGN_TOP_LEFT, 3, 3);
  lv_label_set_text(s_lbl, "Auxiliary…");
  tdisplays3_display_unlock();
  (void)lv_timer_create(ui_timer, 200, NULL);

  adc_oneshot_unit_init_cfg_t acfg = {
      .unit_id = (CONFIG_POT_ADC_UNIT == 2) ? ADC_UNIT_2 : ADC_UNIT_1,
      .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
  };
  if (adc_oneshot_new_unit(&acfg, &s_adc) == ESP_OK) {
    adc_oneshot_chan_cfg_t chc = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    adc_channel_t ch;
    adc_unit_t uu;
    if (adc_oneshot_io_to_channel(CONFIG_POT_GPIO_NUM, &uu, &ch) == ESP_OK &&
        adc_oneshot_config_channel(s_adc, ch, &chc) == ESP_OK) {
      ESP_LOGI(TAG, "ADC on GPIO %d", CONFIG_POT_GPIO_NUM);
    }
  }

  if (can_init(can_rx_cb) != ESP_OK) {
    ESP_LOGE(TAG, "CAN failed");
    return;
  }

  xTaskCreate(gps_uart_task, "gps_uart", 4096, NULL, 5, NULL);
  xTaskCreate(pot_tx_task, "pot_tx", 4096, NULL, 6, NULL);
}
