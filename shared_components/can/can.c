#include "can.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

#define LOG(...) ESP_LOGI("can", __VA_ARGS__)

static can_rx_cb_t given_can_rx_cb = NULL;
static twai_node_handle_t s_node_hdl = NULL;

static bool twai_rx_cb(twai_node_handle_t handle,
                       const twai_rx_done_event_data_t *edata, void *user_ctx) {
  (void)edata;
  (void)user_ctx;
  assert(given_can_rx_cb != NULL &&
         "No given CAN rx callback (must call `can_init()`)");

  uint8_t recv_buff[8];
  twai_frame_t rx_frame = {
      .buffer = recv_buff,
      .buffer_len = sizeof(recv_buff),
  };
  if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) {
    return given_can_rx_cb(recv_buff, rx_frame.header.id,
                           rx_frame.header.timestamp);
  }
  return false;
}

void can_init(can_rx_cb_t can_rx_cb) {
  given_can_rx_cb = can_rx_cb;

  twai_onchip_node_config_t node_config = {
      .io_cfg.tx = CONFIG_CANTX,    // TWAI TX GPIO pin
      .io_cfg.rx = CONFIG_CANRX,    // TWAI RX GPIO pin
      .bit_timing.bitrate = 200000, // 200 kbps bitrate
      .tx_queue_depth = 5,          // Transmit queue depth set to 5
  };
  // Create a new TWAI controller driver instance
  ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &s_node_hdl));

  if (given_can_rx_cb != NULL) {
    twai_event_callbacks_t user_cbs = {
        .on_rx_done = twai_rx_cb,
    };
    ESP_ERROR_CHECK(
        twai_node_register_event_callbacks(s_node_hdl, &user_cbs, NULL));
    LOG("CAN rx callback function set");
  }

  // Start the TWAI controller
  ESP_ERROR_CHECK(twai_node_enable(s_node_hdl));

  twai_node_status_t status_ret;
  twai_node_record_t record;
  twai_node_get_info(s_node_hdl, &status_ret, &record);
  LOG("Initialised CAN");
}

bool can_tx(uint32_t id, const uint8_t *data, uint8_t len) {
  if (s_node_hdl == NULL) {
    ESP_LOGE("can", "CAN not initialized");
    return false;
  }
  if (len > 8) {
    ESP_LOGE("can", "CAN data length exceeds 8 bytes");
    return false;
  }

  uint8_t tx_buff[8] = {0};
  if (data != NULL && len > 0) {
    memcpy(tx_buff, data, len);
  }

  twai_frame_t tx_frame = {
      .header = {
          .id = id,
          .dlc = len,
          .ide = 0, // Standard frame (11-bit ID)
          .rtr = 0, // Data frame
      },
      .buffer = tx_buff,
      .buffer_len = len,
  };

  esp_err_t ret = twai_node_transmit(s_node_hdl, &tx_frame, pdMS_TO_TICKS(100));
  if (ret != ESP_OK) {
    ESP_LOGE("can", "CAN tx failed: %s", esp_err_to_name(ret));
    return false;
  }
  return true;
}
