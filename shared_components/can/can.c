#include "can.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"

static can_rx_cb_t given_can_rx_cb = NULL;
static twai_node_handle_t s_node_hdl = NULL;

// Simple frame buffer pool - reuse persistent buffers
#define FRAME_POOL_SIZE 5
static struct {
  uint8_t buffer[8];
  twai_frame_t frame;
} s_frame_pool[FRAME_POOL_SIZE];

static uint8_t s_frame_idx = 0;  // Round-robin through pool
static SemaphoreHandle_t s_frame_pool_mutex = NULL;

// Diagnostics
static uint32_t s_tx_attempts = 0;
static uint32_t s_tx_failures = 0;

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

  s_frame_pool_mutex = xSemaphoreCreateMutex();
  if (s_frame_pool_mutex == NULL) {
    ESP_LOGE("can", "Failed to create frame pool mutex");
    return;
  }

  twai_onchip_node_config_t node_config = {
      .io_cfg.tx = CONFIG_CANTX,    // TWAI TX GPIO pin
      .io_cfg.rx = CONFIG_CANRX,    // TWAI RX GPIO pin
      .bit_timing.bitrate = CONFIG_CAN_BITRATE,  // Bitrate from kconfig
      .tx_queue_depth = CONFIG_CAN_TX_QUEUE_DEPTH,  // Queue depth from kconfig
  };
  // Create a new TWAI controller driver instance
  ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &s_node_hdl));

  twai_event_callbacks_t user_cbs = {0};
  if (given_can_rx_cb != NULL) {
    user_cbs.on_rx_done = twai_rx_cb;
    ESP_LOGI("can", "CAN rx callback function set");
  }
  ESP_ERROR_CHECK(
      twai_node_register_event_callbacks(s_node_hdl, &user_cbs, NULL));

  // Start the TWAI controller
  ESP_ERROR_CHECK(twai_node_enable(s_node_hdl));

  ESP_LOGI("can", "CAN initialized (fire-and-forget mode, bitrate=%d, queue=%d)", 
           CONFIG_CAN_BITRATE, CONFIG_CAN_TX_QUEUE_DEPTH);
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

  // Get next buffer from round-robin pool
  if (xSemaphoreTake(s_frame_pool_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
    s_tx_failures++;
    return false;  // Just skip if can't acquire quickly
  }

  // Round-robin through pool
  uint8_t idx = s_frame_idx;
  s_frame_idx = (s_frame_idx + 1) % FRAME_POOL_SIZE;
  xSemaphoreGive(s_frame_pool_mutex);

  // Prepare frame in persistent pool buffer
  memset(s_frame_pool[idx].buffer, 0, sizeof(s_frame_pool[idx].buffer));
  if (data != NULL && len > 0) {
    memcpy(s_frame_pool[idx].buffer, data, len);
  }

  s_frame_pool[idx].frame = (twai_frame_t){
      .header = {
          .id = id,
          .dlc = len,
          .ide = 0, // Standard frame (11-bit ID)
          .rtr = 0, // Data frame
      },
      .buffer = s_frame_pool[idx].buffer,
      .buffer_len = len,
  };

  s_tx_attempts++;
  // Fire-and-forget: try to transmit with 1ms timeout, skip silently if queue full
  esp_err_t ret = twai_node_transmit(s_node_hdl, &s_frame_pool[idx].frame, pdMS_TO_TICKS(1));
  if (ret != ESP_OK) {
    s_tx_failures++;
    if (ret == ESP_ERR_NO_MEM) {
      // Queue full - expected during load, will retry next cycle with newer data
      // Log this periodically (every 100 attempts when failing)
      if ((s_tx_attempts % 100) == 0 && s_tx_failures > 0) {
        ESP_LOGD("can", "CAN tx queue full (ID 0x%x, failures: %u/%u)", 
                 id, s_tx_failures, s_tx_attempts);
      }
      return false;
    }
    ESP_LOGE("can", "CAN tx failed (ID 0x%x): %s", id, esp_err_to_name(ret));
    return false;
  }
  return true;
}

void can_get_tx_stats(uint32_t *attempts, uint32_t *failures) {
  if (attempts != NULL) {
    *attempts = s_tx_attempts;
  }
  if (failures != NULL) {
    *failures = s_tx_failures;
  }
}
