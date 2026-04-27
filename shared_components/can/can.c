#include "can.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "can";

static can_rx_cb_t s_given_can_rx_cb = NULL;
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

/* ---------- TX worker ---------- */
typedef struct {
  uint32_t id;
  uint8_t data[8];
  uint8_t len;
} can_tx_req_t;

static QueueHandle_t s_tx_req_queue = NULL;
static TaskHandle_t s_tx_task_hdl = NULL;

/* ---------- RX worker ---------- */
typedef struct {
  uint8_t buffer[8];
  uint32_t id;
  uint64_t timestamp;
  uint8_t len;
} can_rx_msg_t;

static QueueHandle_t s_rx_queue = NULL;
static TaskHandle_t s_rx_task_hdl = NULL;

/* Diagnostics */
static uint32_t s_tx_attempts = 0;
static uint32_t s_tx_failures = 0;

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
  uint8_t buf[8];
  twai_frame_t rx_frame = {
      .buffer = buf,
      .buffer_len = sizeof(buf),
  };
  if (twai_node_receive_from_isr(handle, &rx_frame) != ESP_OK) {
    return false;
  }
  can_rx_msg_t msg;
  memset(&msg, 0, sizeof(msg));
  uint8_t dlen = (rx_frame.buffer_len > 8) ? 8 : (uint8_t)rx_frame.buffer_len;
  memcpy(msg.buffer, rx_frame.buffer, dlen);
  msg.id = rx_frame.header.id;
  msg.timestamp = rx_frame.header.timestamp;
  msg.len = dlen;
  BaseType_t yield = pdFALSE;
  xQueueSendFromISR(s_rx_queue, &msg, &yield);
  return yield == pdTRUE;
}

/* ---------- Worker tasks ---------- */
static void can_tx_task(void *arg) {
  (void)arg;
  can_tx_req_t req;
  for (;;) {
    if (xQueueReceive(s_tx_req_queue, &req, portMAX_DELAY) != pdTRUE) {
      continue;
    }
    can_frame_buf_t *fb = NULL;
    if (xQueueReceive(s_free_frame_queue, &fb, portMAX_DELAY) != pdTRUE) {
      continue;
    }
    if (req.len > 8) {
      req.len = 8;
    }
    if (req.len > 0) {
      memcpy(fb->buffer, req.data, req.len);
    } else {
      memset(fb->buffer, 0, sizeof(fb->buffer));
    }
    fb->frame.header = (twai_frame_header_t){
        .id = req.id,
        .dlc = req.len,
        .ide = 0,
        .rtr = 0,
    };
    fb->frame.buffer = fb->buffer;
    fb->frame.buffer_len = req.len;

    esp_err_t ret = twai_node_transmit(s_node_hdl, &fb->frame, portMAX_DELAY);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "CAN tx failed (ID 0x%x): %s", req.id, esp_err_to_name(ret));
      xQueueSend(s_free_frame_queue, &fb, 0);
    }
  }
}

static void can_rx_task(void *arg) {
  (void)arg;
  can_rx_msg_t msg;
  for (;;) {
    if (xQueueReceive(s_rx_queue, &msg, portMAX_DELAY) != pdTRUE) {
      continue;
    }
    if (s_given_can_rx_cb != NULL) {
      s_given_can_rx_cb(msg.buffer, msg.id, msg.timestamp);
    }
  }
}

/* ---------- Public API ---------- */
esp_err_t can_init(can_rx_cb_t can_rx_cb) {
  if (s_node_hdl != NULL) {
    ESP_LOGW(TAG, "CAN already initialised");
    return ESP_ERR_INVALID_STATE;
  }

  s_given_can_rx_cb = can_rx_cb;

  s_free_frame_queue = xQueueCreate(CAN_FRAME_POOL_SIZE, sizeof(can_frame_buf_t *));
  if (s_free_frame_queue == NULL) {
    return ESP_ERR_NO_MEM;
  }

  s_tx_req_queue = xQueueCreate(CONFIG_CAN_TX_QUEUE_DEPTH, sizeof(can_tx_req_t));
  if (s_tx_req_queue == NULL) {
    vQueueDelete(s_free_frame_queue);
    s_free_frame_queue = NULL;
    return ESP_ERR_NO_MEM;
  }

  s_rx_queue = xQueueCreate(CONFIG_CAN_TX_QUEUE_DEPTH, sizeof(can_rx_msg_t));
  if (s_rx_queue == NULL) {
    vQueueDelete(s_tx_req_queue);
    vQueueDelete(s_free_frame_queue);
    s_tx_req_queue = NULL;
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
  };
  esp_err_t ret = twai_new_node_onchip(&node_config, &s_node_hdl);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "twai_new_node_onchip failed: %s", esp_err_to_name(ret));
    goto cleanup;
  }

  twai_event_callbacks_t user_cbs = {0};
  user_cbs.on_tx_done = can_tx_done_cb;
  if (s_given_can_rx_cb != NULL) {
    user_cbs.on_rx_done = can_rx_done_cb;
    ESP_LOGI(TAG, "CAN rx callback registered");
  }
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

  BaseType_t task_ret = xTaskCreate(can_tx_task, "can_tx", 4096, NULL, 5, &s_tx_task_hdl);
  if (task_ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create can_tx task");
    ret = ESP_ERR_NO_MEM;
    goto cleanup;
  }

  if (s_given_can_rx_cb != NULL) {
    task_ret = xTaskCreate(can_rx_task, "can_rx", 8192, NULL, 5, &s_rx_task_hdl);
    if (task_ret != pdPASS) {
      ESP_LOGE(TAG, "Failed to create can_rx task");
      ret = ESP_ERR_NO_MEM;
      goto cleanup;
    }
  }

  ESP_LOGI(TAG, "CAN initialised (bitrate=%d, queue=%d)", CONFIG_CAN_BITRATE,
           CONFIG_CAN_TX_QUEUE_DEPTH);
  return ESP_OK;

cleanup:
  if (s_tx_task_hdl != NULL) {
    vTaskDelete(s_tx_task_hdl);
    s_tx_task_hdl = NULL;
  }
  if (s_rx_task_hdl != NULL) {
    vTaskDelete(s_rx_task_hdl);
    s_rx_task_hdl = NULL;
  }
  if (s_node_hdl != NULL) {
    twai_node_disable(s_node_hdl);
    twai_node_delete(s_node_hdl);
    s_node_hdl = NULL;
  }
  if (s_tx_req_queue != NULL) {
    vQueueDelete(s_tx_req_queue);
    s_tx_req_queue = NULL;
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

bool can_tx(uint32_t id, const uint8_t *data, uint8_t len) {
  if (s_node_hdl == NULL || s_tx_req_queue == NULL) {
    ESP_LOGE(TAG, "CAN not initialised");
    return false;
  }
  if (len > 8) {
    ESP_LOGE(TAG, "CAN data length exceeds 8 bytes");
    return false;
  }

  twai_node_status_t status;
  if (twai_node_get_info(s_node_hdl, &status, NULL) == ESP_OK) {
    if (status.state == TWAI_ERROR_BUS_OFF) {
      twai_node_recover(s_node_hdl);
      return false;
    }
  }

  can_tx_req_t req = {.id = id, .len = len};
  if (data != NULL && len > 0) {
    memcpy(req.data, data, len);
  }

  s_tx_attempts++;
  if (xQueueSend(s_tx_req_queue, &req, pdMS_TO_TICKS(10)) != pdTRUE) {
    s_tx_failures++;
    if ((s_tx_attempts % 100) == 0) {
      ESP_LOGD(TAG, "CAN tx request queue full (ID 0x%x)", id);
    }
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

esp_err_t can_deinit(void) {
  if (s_node_hdl == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  if (s_tx_task_hdl != NULL) {
    vTaskDelete(s_tx_task_hdl);
    s_tx_task_hdl = NULL;
  }
  if (s_rx_task_hdl != NULL) {
    vTaskDelete(s_rx_task_hdl);
    s_rx_task_hdl = NULL;
  }

  twai_node_disable(s_node_hdl);
  twai_node_delete(s_node_hdl);
  s_node_hdl = NULL;

  if (s_tx_req_queue != NULL) {
    vQueueDelete(s_tx_req_queue);
    s_tx_req_queue = NULL;
  }
  if (s_rx_queue != NULL) {
    vQueueDelete(s_rx_queue);
    s_rx_queue = NULL;
  }
  if (s_free_frame_queue != NULL) {
    vQueueDelete(s_free_frame_queue);
    s_free_frame_queue = NULL;
  }

  s_given_can_rx_cb = NULL;
  s_tx_attempts = 0;
  s_tx_failures = 0;
  ESP_LOGI(TAG, "CAN deinitialised");
  return ESP_OK;
}
