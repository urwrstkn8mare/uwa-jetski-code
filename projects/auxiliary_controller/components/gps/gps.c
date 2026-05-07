#include "gps.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "can.h"
#include "can_ids.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwgps/lwgps.h"
#include "status_ui.h"

_Static_assert(sizeof(lwgps_float_t) == sizeof(float), "lwgps float must be 32-bit");

static const char *TAG = "gps";
static SemaphoreHandle_t s_mutex;
static lwgps_t s_gps;
static uint32_t s_parse_ok;
static uint32_t s_parse_fail;
static bool s_started;

static void publish_position(void) {
  uint8_t payload[sizeof(float) * 2];
  memcpy(&payload[0], &s_gps.latitude, sizeof(float));
  memcpy(&payload[sizeof(float)], &s_gps.longitude, sizeof(float));
  (void)can_tx(CAN_ID_GPS_POSITION, payload, sizeof(payload));
}

static void publish_velocity(void) {
  uint8_t payload[sizeof(float) * 2];
  memcpy(&payload[0], &s_gps.speed, sizeof(float));
  memcpy(&payload[sizeof(float)], &s_gps.course, sizeof(float));
  (void)can_tx(CAN_ID_GPS_VELOCITY, payload, sizeof(payload));
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

  esp_err_t ret = uart_driver_install(port, 4096, 0, 0, NULL, 0);
  ESP_GOTO_ON_ERROR(ret, out, TAG, "uart_driver_install failed");
  ret = uart_param_config(port, &uc);
  ESP_GOTO_ON_ERROR(ret, out, TAG, "uart_param_config failed");
  int tx = CONFIG_GPS_UART_TX_GPIO;
  if (tx < 0) {
    tx = UART_PIN_NO_CHANGE;
  }
  ret = uart_set_pin(port, tx, CONFIG_GPS_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  ESP_GOTO_ON_ERROR(ret, out, TAG, "uart_set_pin failed");
  uart_flush_input(port);

  lwgps_init(&s_gps);

  for (;;) {
    uint8_t buf[384];
    int n = uart_read_bytes(port, buf, sizeof(buf), pdMS_TO_TICKS(120));
    if (n <= 0) {
      continue;
    }

    if (s_mutex == NULL || xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
      continue;
    }

    lwgps_process(&s_gps, buf, (size_t)n, on_sentence);

    status_ui_update("GPS",
                     "fx%u q%u sat%u ok%u x%u lat%.6f lon%.6f spd%.2fkt hdg%.2f",
                     (unsigned)((s_gps.is_valid || s_gps.fix > 0) ? 1 : 0),
                     (unsigned)s_gps.fix,
                     (unsigned)s_gps.sats_in_use,
                     (unsigned)s_parse_ok,
                     (unsigned)s_parse_fail,
                     (double)s_gps.latitude,
                     (double)s_gps.longitude,
                     (double)s_gps.speed,
                     (double)s_gps.course);

    xSemaphoreGive(s_mutex);
  }

out:
  ESP_LOGE(TAG, "GPS UART task exiting: %s", esp_err_to_name(ret));
  vTaskDelete(NULL);
}

esp_err_t gps_init(void) {
  if (s_started) {
    return ESP_OK;
  }

  s_mutex = xSemaphoreCreateMutex();
  if (s_mutex == NULL) {
    ESP_LOGE(TAG, "mutex alloc failed");
    return ESP_ERR_NO_MEM;
  }

  if (xTaskCreate(gps_uart_task, "gps_uart", 8192, NULL, 5, NULL) != pdPASS) {
    ESP_LOGE(TAG, "task create failed");
    vSemaphoreDelete(s_mutex);
    s_mutex = NULL;
    return ESP_ERR_NO_MEM;
  }

  s_started = true;
  ESP_LOGI(TAG, "GPS UART%d RX GPIO %d %u baud", CONFIG_GPS_UART_PORT_NUM, CONFIG_GPS_UART_RX_GPIO,
           (unsigned)CONFIG_GPS_UART_BAUD);
  return ESP_OK;
}
