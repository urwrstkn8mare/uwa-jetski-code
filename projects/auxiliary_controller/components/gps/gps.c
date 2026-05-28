#include "gps.h"

#include <stdbool.h>

#include "can.h"
#include "can_ids.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwgps/lwgps.h"
#include "status_ui.h"

_Static_assert(sizeof(lwgps_float_t) == sizeof(float), "lwgps float must be 32-bit");

static const char *TAG = "gps";
static lwgps_t s_gps;
static bool s_started;

static void update_position(void) {
  can_gps_position_t pos = { .lat_deg = s_gps.latitude, .lon_deg = s_gps.longitude };
  (void)can_tx(CAN_ID_GPS_POSITION, (const uint8_t *)&pos, sizeof(pos));
  status_ui_update("GPS pos", "%.6f %.6f", (double)s_gps.latitude, (double)s_gps.longitude);
}

static void update_velocity(void) {
  can_gps_velocity_t vel = { .speed_knots = s_gps.speed, .course_deg = s_gps.course };
  (void)can_tx(CAN_ID_GPS_VELOCITY, (const uint8_t *)&vel, sizeof(vel));
  status_ui_update("GPS vel", "%.2fkt %.0f°", (double)s_gps.speed, (double)s_gps.course);
}

static void on_sentence(lwgps_statement_t res) {
  if (res == STAT_RMC && s_gps.is_valid) {
    update_position();
    update_velocity();
  } else if (res == STAT_GGA && s_gps.fix > 0) {
    update_position();
    status_ui_update("GPS fix", "fx%u sat%u", (unsigned)s_gps.fix, (unsigned)s_gps.sats_in_use);
  }
}

static void gps_uart_task(void *arg) {
  (void)arg;
  const uart_port_t port = (uart_port_t)CONFIG_GPS_UART_PORT_NUM;

  uart_config_t uc = {
      .baud_rate  = CONFIG_GPS_UART_BAUD,
      .data_bits  = UART_DATA_8_BITS,
      .parity     = UART_PARITY_DISABLE,
      .stop_bits  = UART_STOP_BITS_1,
      .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  esp_err_t ret = uart_driver_install(port, 4096, 0, 0, NULL, 0);
  ESP_GOTO_ON_ERROR(ret, out, TAG, "uart_driver_install failed");
  ret = uart_param_config(port, &uc);
  ESP_GOTO_ON_ERROR(ret, out, TAG, "uart_param_config failed");
  int tx = CONFIG_GPS_UART_TX_GPIO;
  if (tx < 0) tx = UART_PIN_NO_CHANGE;
  ret = uart_set_pin(port, tx, CONFIG_GPS_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  ESP_GOTO_ON_ERROR(ret, out, TAG, "uart_set_pin failed");
  uart_flush_input(port);

  lwgps_init(&s_gps);

  for (;;) {
    uint8_t buf[384];
    int n = uart_read_bytes(port, buf, sizeof(buf), pdMS_TO_TICKS(120));
    if (n <= 0) continue;

    lwgps_process(&s_gps, buf, (size_t)n, on_sentence);
  }

out:
  ESP_LOGE(TAG, "GPS UART task exiting: %s", esp_err_to_name(ret));
  vTaskDelete(NULL);
}

esp_err_t gps_init(void) {
  if (s_started) return ESP_OK;

  if (xTaskCreate(gps_uart_task, "gps_uart", 8192, NULL, 5, NULL) != pdPASS) {
    ESP_LOGE(TAG, "task create failed");
    return ESP_ERR_NO_MEM;
  }

  s_started = true;
  ESP_LOGI(TAG, "GPS UART%d RX GPIO%d %ubaud",
           CONFIG_GPS_UART_PORT_NUM, CONFIG_GPS_UART_RX_GPIO, (unsigned)CONFIG_GPS_UART_BAUD);
  return ESP_OK;
}
