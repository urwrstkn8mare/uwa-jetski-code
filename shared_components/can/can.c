#include "can.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

#define LOG(...) ESP_LOGI("can", __VA_ARGS__)

#define TX_FRAME_POOL_SIZE 5

typedef struct {
  uint8_t buffer[8];
  twai_frame_t frame;
  bool in_use;
} tx_frame_slot_t;

static can_rx_cb_t given_can_rx_cb = NULL;
static twai_node_handle_t s_node_hdl = NULL;
static tx_frame_slot_t s_tx_pool[TX_FRAME_POOL_SIZE];
static SemaphoreHandle_t s_tx_pool_mutex = NULL;

static bool twai_tx_done_cb(twai_node_handle_t handle,
                            const twai_tx_done_event_data_t *edata,
                            void *user_ctx) {
  (void)handle;
  (void)user_ctx;
  if (edata == NULL || edata->done_tx_frame == NULL) {
    return false;
  }
  // Find the slot matching the completed frame and mark it free
  for (int i = 0; i < TX_FRAME_POOL_SIZE; i++) {
    if (&s_tx_pool[i].frame == edata->done_tx_frame) {
      if (xSemaphoreTakeFromISR(s_tx_pool_mutex, NULL) == pdTRUE) {
        s_tx_pool[i].in_use = false;
        xSemaphoreGiveFromISR(s_tx_pool_mutex, NULL);
      }
      break;
    }
  }
  return false;
}

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

  s_tx_pool_mutex = xSemaphoreCreateMutex();
  if (s_tx_pool_mutex == NULL) {
    ESP_LOGE("can", "Failed to create TX pool mutex");
    return;
  }
  memset(s_tx_pool, 0, sizeof(s_tx_pool));

  twai_onchip_node_config_t node_config = {
      .io_cfg.tx = CONFIG_CANTX,    // TWAI TX GPIO pin
      .io_cfg.rx = CONFIG_CANRX,    // TWAI RX GPIO pin
      .bit_timing.bitrate = 200000, // 200 kbps bitrate
      .tx_queue_depth = 5,          // Transmit queue depth set to 5
  };
  // Create a new TWAI controller driver instance
  ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &s_node_hdl));

  twai_event_callbacks_t user_cbs = {0};
  user_cbs.on_tx_done = twai_tx_done_cb;
  if (given_can_rx_cb != NULL) {
    user_cbs.on_rx_done = twai_rx_cb;
    LOG("CAN rx callback function set");
  }
  ESP_ERROR_CHECK(
      twai_node_register_event_callbacks(s_node_hdl, &user_cbs, NULL));

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

  twai_node_status_t status;
  if (twai_node_get_info(s_node_hdl, &status, NULL) == ESP_OK) {
    if (status.state == TWAI_ERROR_BUS_OFF) {
      twai_node_recover(s_node_hdl);
      return false;
    }
  }

  if (xSemaphoreTake(s_tx_pool_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    ESP_LOGE("can", "CAN tx pool mutex timeout");
    return false;
  }

  tx_frame_slot_t *slot = NULL;
  for (int i = 0; i < TX_FRAME_POOL_SIZE; i++) {
    if (!s_tx_pool[i].in_use) {
      slot = &s_tx_pool[i];
      slot->in_use = true;
      break;
    }
  }

  if (slot == NULL) {
    xSemaphoreGive(s_tx_pool_mutex);
    ESP_LOGE("can", "CAN tx frame pool exhausted");
    return false;
  }

  memset(slot->buffer, 0, sizeof(slot->buffer));
  if (data != NULL && len > 0) {
    memcpy(slot->buffer, data, len);
  }

  slot->frame = (twai_frame_t){
      .header = {
          .id = id,
          .dlc = len,
          .ide = 0, // Standard frame (11-bit ID)
          .rtr = 0, // Data frame
      },
      .buffer = slot->buffer,
      .buffer_len = len,
  };

  // Release mutex before blocking transmit so ISR can acquire it for on_tx_done
  xSemaphoreGive(s_tx_pool_mutex);

  esp_err_t ret = twai_node_transmit(s_node_hdl, &slot->frame, pdMS_TO_TICKS(100));
  if (ret != ESP_OK) {
    // Mark slot free on failure
    if (xSemaphoreTake(s_tx_pool_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      slot->in_use = false;
      xSemaphoreGive(s_tx_pool_mutex);
    }
    ESP_LOGE("can", "CAN tx failed: %s", esp_err_to_name(ret));
    return false;
  }
  return true;
}
