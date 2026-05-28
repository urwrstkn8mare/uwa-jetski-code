#include "can.h"
#include "status_ui.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "hal/twai_types.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <string.h>

static const char *TAG = "can";

#define CAN_MAX_RX_CBS 8
static can_rx_cb_t s_rx_cbs[CAN_MAX_RX_CBS];
static uint8_t s_rx_cb_count = 0;

static twai_node_handle_t s_node_hdl = NULL;

/* ---------- Frame pool for safe asynchronous TX ----------
 * The TWAI driver queues twai_frame_t * pointers internally.
 * Frame memory must remain valid until on_tx_done fires.
 * We allocate a static pool and recycle via a free-queue.
 */
#define CAN_FRAME_POOL_SIZE (CONFIG_CAN_TX_QUEUE_DEPTH + 4)

typedef struct {
  twai_frame_t frame;
  uint8_t buffer[8];
} can_frame_buf_t;

static can_frame_buf_t s_frame_pool[CAN_FRAME_POOL_SIZE];
static QueueHandle_t s_free_frame_queue = NULL;

/* ---------- RX worker ---------- */
typedef struct {
  uint8_t buffer[8];
  uint32_t id;
  uint64_t timestamp;
  uint8_t len;
} can_rx_msg_t;

static QueueHandle_t s_rx_queue = NULL;
static TaskHandle_t s_rx_task_hdl = NULL;
static TaskHandle_t s_diag_task_hdl = NULL;

/* ---------- ISR callbacks (must be in IRAM) ---------- */
static IRAM_ATTR bool can_tx_done_cb(twai_node_handle_t handle,
                                      const twai_tx_done_event_data_t *edata,
                                      void *user_ctx) {
  (void)handle;
  (void)user_ctx;
  if (s_free_frame_queue == NULL) {
    return false;
  }
  const twai_frame_t *done_frame = edata->done_tx_frame;
  can_frame_buf_t *fb = (can_frame_buf_t *)done_frame; /* frame is first member */
  BaseType_t yield = pdFALSE;
  xQueueSendFromISR(s_free_frame_queue, &fb, &yield);
  return yield == pdTRUE;
}

static IRAM_ATTR bool can_rx_done_cb(twai_node_handle_t handle,
                                      const twai_rx_done_event_data_t *edata,
                                      void *user_ctx) {
  (void)edata;
  (void)user_ctx;
  if (s_rx_queue == NULL) {
    return false;
  }
  can_rx_msg_t msg = {0};
  twai_frame_t rx_frame = {
      .buffer = msg.buffer,
      .buffer_len = sizeof(msg.buffer),
  };
  if (twai_node_receive_from_isr(handle, &rx_frame) != ESP_OK) {
    return false;
  }
  msg.id = rx_frame.header.id;
  msg.timestamp = rx_frame.header.timestamp;
  msg.len = (rx_frame.buffer_len > 8) ? 8 : (uint8_t)rx_frame.buffer_len;
  BaseType_t yield = pdFALSE;
  xQueueSendFromISR(s_rx_queue, &msg, &yield);
  return yield == pdTRUE;
}

/* ---------- Worker tasks ---------- */
static void can_rx_task(void *arg) {
  (void)arg;
  can_rx_msg_t msg;
  for (;;) {
    if (xQueueReceive(s_rx_queue, &msg, portMAX_DELAY) != pdTRUE) {
      continue;
    }
    for (uint8_t i = 0; i < s_rx_cb_count; i++) {
      s_rx_cbs[i](msg.buffer, msg.id, msg.timestamp);
    }
  }
}

static void can_diag_task(void *arg) {
  (void)arg;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(2500));
    if (s_node_hdl == NULL) continue;
    twai_node_status_t st = {0};
    if (twai_node_get_info(s_node_hdl, &st, NULL) != ESP_OK) continue;
    if (st.state == TWAI_ERROR_BUS_OFF) twai_node_recover(s_node_hdl);
    const char *state;
    switch (st.state) {
      case TWAI_ERROR_ACTIVE:  state = "RUN";     break;
      case TWAI_ERROR_WARNING: state = "WARN";    break;
      case TWAI_ERROR_PASSIVE: state = "PASSIVE"; break;
      case TWAI_ERROR_BUS_OFF: state = "BUS_OFF"; break;
      default:                 state = "?";       break;
    }
    status_ui_update("CAN", "%s", state);
  }
}

/* ---------- Public API ---------- */
esp_err_t can_register_rx_cb(can_rx_cb_t cb) {
  if (cb == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (s_rx_cb_count >= CAN_MAX_RX_CBS) {
    ESP_LOGE(TAG, "RX callback table full (%d)", CAN_MAX_RX_CBS);
    return ESP_ERR_NO_MEM;
  }
  s_rx_cbs[s_rx_cb_count++] = cb;
  return ESP_OK;
}

esp_err_t can_init(void) {
  if (s_node_hdl != NULL) {
    ESP_LOGW(TAG, "CAN already initialised");
    return ESP_ERR_INVALID_STATE;
  }

  s_free_frame_queue = xQueueCreate(CAN_FRAME_POOL_SIZE, sizeof(can_frame_buf_t *));
  if (s_free_frame_queue == NULL) {
    return ESP_ERR_NO_MEM;
  }

  s_rx_queue = xQueueCreate(CONFIG_CAN_TX_QUEUE_DEPTH, sizeof(can_rx_msg_t));
  if (s_rx_queue == NULL) {
    vQueueDelete(s_free_frame_queue);
    s_free_frame_queue = NULL;
    return ESP_ERR_NO_MEM;
  }

  for (int i = 0; i < CAN_FRAME_POOL_SIZE; i++) {
    s_frame_pool[i].frame.buffer = s_frame_pool[i].buffer;
    s_frame_pool[i].frame.buffer_len = sizeof(s_frame_pool[i].buffer);
    can_frame_buf_t *ptr = &s_frame_pool[i];
    xQueueSend(s_free_frame_queue, &ptr, 0);
  }

  twai_onchip_node_config_t node_config = {
      .io_cfg.tx = CONFIG_CANTX,
      .io_cfg.rx = CONFIG_CANRX,
      .bit_timing.bitrate = CONFIG_CAN_BITRATE,
      .tx_queue_depth = CONFIG_CAN_TX_QUEUE_DEPTH,
      .fail_retry_cnt = 4,
  };
  esp_err_t ret = twai_new_node_onchip(&node_config, &s_node_hdl);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "twai_new_node_onchip failed: %s", esp_err_to_name(ret));
    goto cleanup;
  }

  twai_mask_filter_config_t accept_all_std = {
      .id = 0,
      .mask = 0,
      .is_ext = 0,
      .no_classic = 0,
      .no_fd = 1,
      .dual_filter = 0,
  };
  ret = twai_node_config_mask_filter(s_node_hdl, 0, &accept_all_std);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "twai_node_config_mask_filter failed: %s", esp_err_to_name(ret));
    goto cleanup;
  }

  twai_event_callbacks_t user_cbs = {0};
  user_cbs.on_tx_done = can_tx_done_cb;
  user_cbs.on_rx_done = can_rx_done_cb;
  ret = twai_node_register_event_callbacks(s_node_hdl, &user_cbs, NULL);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "twai_node_register_event_callbacks failed: %s", esp_err_to_name(ret));
    goto cleanup;
  }

  ret = twai_node_enable(s_node_hdl);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "twai_node_enable failed: %s", esp_err_to_name(ret));
    goto cleanup;
  }

  BaseType_t task_ret = xTaskCreate(can_rx_task, "can_rx", 8192, NULL, 5, &s_rx_task_hdl);
  if (task_ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create can_rx task");
    ret = ESP_ERR_NO_MEM;
    goto cleanup;
  }

  ESP_LOGI(TAG, "TWAI up: %dkbps Tx%u Rx%u",
           CONFIG_CAN_BITRATE / 1000, (unsigned)CONFIG_CANTX, (unsigned)CONFIG_CANRX);
  xTaskCreate(can_diag_task, "can_diag", 4096, NULL, 3, &s_diag_task_hdl);
  return ESP_OK;

cleanup:
  if (s_rx_task_hdl != NULL) {
    vTaskDelete(s_rx_task_hdl);
    s_rx_task_hdl = NULL;
  }
  if (s_node_hdl != NULL) {
    twai_node_disable(s_node_hdl);
    twai_node_delete(s_node_hdl);
    s_node_hdl = NULL;
  }
  if (s_rx_queue != NULL) {
    vQueueDelete(s_rx_queue);
    s_rx_queue = NULL;
  }
  if (s_free_frame_queue != NULL) {
    vQueueDelete(s_free_frame_queue);
    s_free_frame_queue = NULL;
  }
  return ret;
}

esp_err_t can_tx(uint32_t id, const uint8_t *data, uint8_t len) {
  if (s_node_hdl == NULL || s_free_frame_queue == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  if (len > 8) {
    return ESP_ERR_INVALID_SIZE;
  }

  can_frame_buf_t *fb = NULL;
  if (xQueueReceive(s_free_frame_queue, &fb, pdMS_TO_TICKS(10)) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }

  memset(fb->buffer, 0, sizeof(fb->buffer));
  if (len > 0) memcpy(fb->buffer, data, len);
  fb->frame.header = (twai_frame_header_t){.id = id, .dlc = len, .ide = 0, .rtr = 0};
  fb->frame.buffer     = fb->buffer;
  fb->frame.buffer_len = len;

  esp_err_t ret = twai_node_transmit(s_node_hdl, &fb->frame, pdMS_TO_TICKS(10));
  if (ret != ESP_OK) {
    xQueueSend(s_free_frame_queue, &fb, 0);
    static int64_t last_log_us;
    int64_t now = esp_timer_get_time();
    if (now - last_log_us > 5000000 || last_log_us == 0) {
      ESP_LOGW(TAG, "CAN TX failed: %s", esp_err_to_name(ret));
      last_log_us = now;
    }
  }
  return ret;
}

