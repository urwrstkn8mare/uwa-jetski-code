#include "gps.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "can.h"
#include "can_ids.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwgps/lwgps.h"

_Static_assert(sizeof(lwgps_float_t) == sizeof(float), "lwgps float must be 32-bit");

static const char *TAG = "gps";
static SemaphoreHandle_t s_mux;
static lwgps_t s_gps;
static uint32_t s_parse_ok;
static uint32_t s_parse_fail;
static bool s_started;
static status_write_cb_t s_status_write = NULL;
static void *s_status_write_ctx = NULL;

static void publish_position(void) {
  uint8_t payload[sizeof(float) * 2];
  memcpy(&payload[0], &s_gps.latitude, sizeof(float));
  memcpy(&payload[sizeof(float)], &s_gps.longitude, sizeof(float));
  can_tx(CAN_ID_GPS_POSITION, payload, sizeof(payload));
}

static void publish_velocity(void) {
  uint8_t payload[sizeof(float) * 2];
  memcpy(&payload[0], &s_gps.speed, sizeof(float));
  memcpy(&payload[sizeof(float)], &s_gps.course, sizeof(float));
  can_tx(CAN_ID_GPS_VELOCITY, payload, sizeof(payload));
}

static void on_sentence(lwgps_statement_t res) {
  if (res == STAT_CHECKSUM_FAIL) {
    s_parse_fail++;
    return;
  }

  if (res == STAT_RMC && s_gps.is_valid) {
    publish_position();
    publish_velocity();
    s_parse_ok++;
    return;
  }

  if (res == STAT_GGA && s_gps.fix > 0) {
    publish_position();
    s_parse_ok++;
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

  ESP_ERROR_CHECK(uart_driver_install(port, 4096, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(port, &uc));
  int tx = CONFIG_GPS_UART_TX_GPIO;
  if (tx < 0) {
    tx = UART_PIN_NO_CHANGE;
  }
  ESP_ERROR_CHECK(uart_set_pin(port, tx, CONFIG_GPS_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  uart_flush_input(port);

  lwgps_init(&s_gps);

  for (;;) {
    uint8_t buf[384];
    int n = uart_read_bytes(port, buf, sizeof(buf), pdMS_TO_TICKS(120));
    if (n <= 0) {
      continue;
    }

    if (s_mux == NULL || xSemaphoreTake(s_mux, pdMS_TO_TICKS(10)) != pdTRUE) {
      continue;
    }

    lwgps_process(&s_gps, buf, (size_t)n, on_sentence);

    if (s_status_write) {
      float lat = s_gps.latitude;
      float lon = s_gps.longitude;
      float speed = s_gps.speed;
      float course = s_gps.course;
      uint8_t fix = (s_gps.is_valid || s_gps.fix > 0) ? 1 : 0;
      uint8_t quality = s_gps.fix;
      uint8_t sats = s_gps.sats_in_use;
      uint32_t ok = s_parse_ok;
      uint32_t fail = s_parse_fail;

      s_status_write(s_status_write_ctx, "GPS",
                     "fx%u q%u sat%u ok%u x%u lat%.6f lon%.6f spd%.2fkt hdg%.2f",
                     fix, quality, sats, (unsigned)ok, (unsigned)fail,
                     (double)lat, (double)lon,
                     (double)speed, (double)course);
    }

    xSemaphoreGive(s_mux);
  }
}

void gps_init(status_write_cb_t status_write, void *status_write_ctx) {
  s_status_write = status_write;
  s_status_write_ctx = status_write_ctx;

  if (s_started) {
    return;
  }

  s_mux = xSemaphoreCreateMutex();
  if (s_mux == NULL) {
    ESP_LOGE(TAG, "mutex alloc failed");
    return;
  }

  if (xTaskCreate(gps_uart_task, "gps_uart", 8192, NULL, 5, NULL) != pdPASS) {
    ESP_LOGE(TAG, "task create failed");
    vSemaphoreDelete(s_mux);
    s_mux = NULL;
    return;
  }

  s_started = true;
  ESP_LOGI(TAG, "GPS UART%d RX GPIO %d %u baud", CONFIG_GPS_UART_PORT_NUM, CONFIG_GPS_UART_RX_GPIO,
           (unsigned)CONFIG_GPS_UART_BAUD);
}
