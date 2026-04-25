#include "can.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

#define LOG(...) ESP_LOGI("can", __VA_ARGS__)

static can_rx_cb_t given_can_rx_cb = NULL;

static bool twai_rx_cb(twai_node_handle_t handle,
                       const twai_rx_done_event_data_t *edata, void *user_ctx) {
  LOG("CAN received something");
  assert(given_can_rx_cb != NULL &&
         "No given CAN rx callback (must call `can_init()`)");

  uint8_t recv_buff[8];
  twai_frame_t rx_frame = {
      .buffer = recv_buff,
      .buffer_len = sizeof(recv_buff),
  };
  if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) {
    // receive ok, do something here
    return given_can_rx_cb(recv_buff, rx_frame.header.id,
                           rx_frame.header.timestamp);
  }
  return false;
}

void can_init(can_rx_cb_t can_rx_cb) {
  given_can_rx_cb = can_rx_cb;

  twai_node_handle_t node_hdl = NULL;
  twai_onchip_node_config_t node_config = {
      .io_cfg.tx = CONFIG_CANTX,    // TWAI TX GPIO pin
      .io_cfg.rx = CONFIG_CANRX,    // TWAI RX GPIO pin
      .bit_timing.bitrate = 200000, // 200 kbps bitrate
      .tx_queue_depth = 5,          // Transmit queue depth set to 5
  };
  // Create a new TWAI controller driver instance
  ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &node_hdl));

  if (given_can_rx_cb != NULL) {
    twai_event_callbacks_t user_cbs = {
        .on_rx_done = twai_rx_cb,
    };
    ESP_ERROR_CHECK(
        twai_node_register_event_callbacks(node_hdl, &user_cbs, NULL));
    LOG("CAN rx callback function set");
  }

  // Start the TWAI controller
  ESP_ERROR_CHECK(twai_node_enable(node_hdl));

  twai_node_status_t status_ret;
  twai_node_record_t record;
  twai_node_get_info(node_hdl, &status_ret, &record);
  LOG("Initialised CAN");
}
