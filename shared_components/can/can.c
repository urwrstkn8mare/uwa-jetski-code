#include "can.h"

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
#include <stdio.h>
#include <string.h>

static const char *TAG = "can";

static can_rx_cb_t s_given_can_rx_cb = NULL;
static can_status_cb_t s_status_cb = NULL;

void can_register_status_cb(can_status_cb_t cb) {
  s_status_cb = cb;
}
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
static TaskHandle_t s_diag_task_hdl = NULL;

/* Diagnostics */
static uint32_t s_tx_attempts = 0;
static uint32_t s_tx_failures = 0;
static int64_t s_last_can_tx_hw_err_log_us;

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
    memset(fb->buffer, 0, sizeof(fb->buffer));
    if (req.len > 0) {
      memcpy(fb->buffer, req.data, req.len);
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
      s_tx_failures++;
      int64_t now = esp_timer_get_time();
      if (now - s_last_can_tx_hw_err_log_us > 5000000 || s_last_can_tx_hw_err_log_us == 0) {
        ESP_LOGW(TAG, "CAN TX failing (often no transceiver/bus); last err %s ID 0x%" PRIx32,
                 esp_err_to_name(ret), req.id);
        s_last_can_tx_hw_err_log_us = now;
      }
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

static void can_diag_task(void *arg) {
  (void)arg;
  const TickType_t period = pdMS_TO_TICKS(CONFIG_CAN_STATUS_LOG_PERIOD_MS);
  /* Stagger vs. startup banner printed in @ref can_init */
  vTaskDelay(period);
  for (;;) {
    if (can_is_ready()) {
      char line[144];
      (void)can_snprintf_metrics_line(line, sizeof(line));
      ESP_LOGI(TAG, "%s", line);
      if (s_status_cb) {
        char board_line[144];
        (void)can_snprintf_board_status(board_line, sizeof(board_line));
        s_status_cb(board_line);
      }
    }
    vTaskDelay(period);
  }
}

static void can_diag_stop(void) {
  if (s_diag_task_hdl != NULL) {
    TaskHandle_t h = s_diag_task_hdl;
    s_diag_task_hdl = NULL;
    vTaskDelete(h);
  }
}

/* ---------- Public API ---------- */
esp_err_t can_init(can_rx_cb_t can_rx_cb) {
  if (s_node_hdl != NULL) {
    ESP_LOGW(TAG, "CAN already initialised");
    return ESP_ERR_INVALID_STATE;
  }

#ifdef CONFIG_CAN_SKIP_HW
  (void)can_rx_cb;
  ESP_LOGW(TAG, "CAN hardware disabled (CONFIG_CAN_SKIP_HW) — TWAI not started");
  return ESP_ERR_NOT_SUPPORTED;
#endif

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
      /* Cap HW TX retries (not -1) to limit error traffic when the bus is marginal. */
      .fail_retry_cnt = 4,
  };
#ifdef CONFIG_CAN_SELF_TEST
  node_config.flags.enable_self_test = 1;
#endif
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

#ifdef CONFIG_CAN_SELF_TEST
  const int self_test = 1;
#else
  const int self_test = 0;
#endif
  ESP_LOGI(TAG,
           "TWAI up: Tx%u (MCU to transceiver DI) Rx%u (from RO); %dkbps; tx_queue_depth=%u; "
           "self_test=%d",
           (unsigned)CONFIG_CANTX, (unsigned)CONFIG_CANRX, CONFIG_CAN_BITRATE / 1000,
           (unsigned)CONFIG_CAN_TX_QUEUE_DEPTH, self_test);
  if (CONFIG_CAN_STATUS_LOG_PERIOD_MS > 0) {
    if (xTaskCreate(can_diag_task, "can_diag", 3584, NULL, 3, &s_diag_task_hdl) != pdPASS) {
      ESP_LOGW(TAG, "can_diag task not started (OOM?) — periodic status log disabled");
    }
  }
  return ESP_OK;

cleanup:
  can_diag_stop();
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

bool can_is_ready(void) { return s_node_hdl != NULL && s_tx_req_queue != NULL; }

static void fmt_twai_state(twai_error_state_t state, char *dst, size_t dst_len) {
  const char *s = "?";
  switch (state) {
  case TWAI_ERROR_ACTIVE:
    s = "RUN";
    break;
  case TWAI_ERROR_WARNING:
    s = "WARN";
    break;
  case TWAI_ERROR_PASSIVE:
    s = "PASSIVE";
    break;
  case TWAI_ERROR_BUS_OFF:
    s = "BUS_OFF";
    break;
  default:
    s = "UNKNOWN";
    break;
  }
  strncpy(dst, s, dst_len - 1);
  dst[dst_len - 1] = '\0';
}

void can_get_bus_health(can_bus_health_t *out) {
  if (out == NULL) {
    return;
  }
  memset(out, 0, sizeof(*out));
  strncpy(out->state_label, "--", sizeof(out->state_label) - 1);
  if (!can_is_ready()) {
    strncpy(out->state_label, "DOWN", sizeof(out->state_label) - 1);
    return;
  }
  out->controller_started = true;
  twai_node_status_t st = {0};
  twai_node_record_t rec = {0};
  if (twai_node_get_info(s_node_hdl, &st, &rec) != ESP_OK) {
    strncpy(out->state_label, "INFO?", sizeof(out->state_label) - 1);
    return;
  }
  fmt_twai_state(st.state, out->state_label, sizeof(out->state_label));
  out->tx_error_count = st.tx_error_count;
  out->rx_error_count = st.rx_error_count;
  out->bus_error_events = rec.bus_err_num;
}

esp_err_t can_tx(uint32_t id, const uint8_t *data, uint8_t len) {
  if (s_node_hdl == NULL || s_tx_req_queue == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  if (len > 8) {
    ESP_LOGE(TAG, "CAN data length exceeds 8 bytes");
    return ESP_ERR_INVALID_SIZE;
  }

  twai_node_status_t status;
  if (twai_node_get_info(s_node_hdl, &status, NULL) == ESP_OK) {
    if (status.state == TWAI_ERROR_BUS_OFF) {
      twai_node_recover(s_node_hdl);
      return ESP_ERR_INVALID_STATE;
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
    return ESP_ERR_TIMEOUT;
  }
  return ESP_OK;
}

void can_get_tx_stats(uint32_t *attempts, uint32_t *failures) {
  if (attempts != NULL) {
    *attempts = s_tx_attempts;
  }
  if (failures != NULL) {
    *failures = s_tx_failures;
  }
}

int can_snprintf_metrics_line(char *dst, size_t dst_len) {
  if (dst == NULL || dst_len == 0) {
    return -1;
  }
  if (!can_is_ready()) {
    return snprintf(dst, dst_len, "off Tx%u Rx%u txQ%" PRIu32 " fl%" PRIu32 "",
                    (unsigned)CONFIG_CANTX, (unsigned)CONFIG_CANRX,
                    (uint32_t)0, (uint32_t)0);
  }
  can_bus_health_t bh = {0};
  can_get_bus_health(&bh);
  uint32_t qa = 0, qf = 0;
  can_get_tx_stats(&qa, &qf);
  return snprintf(dst, dst_len, "%s Tx%u Rx%u TEC:%u REC:%u bus:%" PRIu32 " txQ%" PRIu32 " fl%" PRIu32 "",
                  bh.controller_started ? bh.state_label : "DOWN",
                  (unsigned)CONFIG_CANTX,
                  (unsigned)CONFIG_CANRX,
                  (unsigned)bh.tx_error_count,
                  (unsigned)bh.rx_error_count,
                  bh.bus_error_events,
                  qa,
                  qf);
}

int can_snprintf_board_status(char *dst, size_t dst_len) {
  if (dst == NULL || dst_len == 0) {
    return -1;
  }
  char inner[136];
  (void)can_snprintf_metrics_line(inner, sizeof(inner));
  return snprintf(dst, dst_len, "CAN %s", inner);
}

esp_err_t can_deinit(void) {
  if (s_node_hdl == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  can_diag_stop();
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
