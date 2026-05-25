#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "esp_err.h"

/*
 * Callback invoked when a CAN frame is received.
 *
 * Called from the CAN RX worker task context (never from ISR context).
 */
typedef void (*can_rx_cb_t)(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp);

/** Optional callback invoked by the diag task with a one-line status string. */
typedef void (*can_status_cb_t)(const char *line);
void can_register_status_cb(can_status_cb_t cb);

/**
 * @brief Initialise CAN (TWAI)
 *
 * Creates internal TX/RX worker tasks and starts the TWAI node.
 *
 * @param[in] can_rx_cb Function to call when CAN frame received. Can be NULL.
 * @return ESP_OK on success, error code on failure.
 */
esp_err_t can_init(can_rx_cb_t can_rx_cb);

typedef struct can_bus_health {
  /** False if @ref can_init never succeeded */
  bool controller_started;
  /** Short controller state ("ACTIVE","WARN","PASSIVE","BUS_OFF",…) */
  char state_label[20];
  uint16_t tx_error_count;
  uint16_t rx_error_count;
  /** Cumulative bus errors since enable (TWAI HAL record) */
  uint32_t bus_error_events;
} can_bus_health_t;

/**
 * @brief Fill TWAI health (error state + counters).
 *
 * Solo node without acks tends toward high TX error counts and BUS_OFF/WARN.
 */
void can_get_bus_health(can_bus_health_t *out);

/** @return true when the TWAI node is created and the TX worker is running — not “another ECU present”. */
bool can_is_ready(void);

/**
 * @brief Transmit a CAN frame
 *
 * @param[in] id: CAN message identifier (11-bit standard ID)
 * @param[in] data: Pointer to data buffer (max 8 bytes)
 * @param[in] len: Data length (0-8)
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if CAN not ready,
 *         ESP_ERR_INVALID_SIZE if len > 8, ESP_ERR_TIMEOUT if TX queue full.
 */
esp_err_t can_tx(uint32_t id, const uint8_t *data, uint8_t len);

/**
 * @brief Get CAN TX diagnostics (attempts and failures)
 *
 * @param[out] attempts: Total transmission attempts
 * @param[out] failures: Total transmission failures
 */
void can_get_tx_stats(uint32_t *attempts, uint32_t *failures);

/**
 * @brief Format TWAI counters + pins + TX enqueue stats into one compact line (no \"CAN \" prefix).
 *
 * Typical output when running: RUN Tx12 Rx13 TEC:0 REC:0 bus_events:… q … f … .
 * Intended for reuse in LVGL labels and periodic serial logs.
 *
 * @return Bytes written excluding the terminating NUL (like snprintf); <0 if dst invalid.
 */
int can_snprintf_metrics_line(char *dst, size_t dst_len);

/** One-line label with \"CAN \" prefix suitable for dashboards (uses @ref can_snprintf_metrics_line). */
int can_snprintf_board_status(char *dst, size_t dst_len);

/**
 * @brief Deinitialise CAN (TWAI)
 *
 * Stops the TWAI node, deletes worker tasks and frees queues.
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialised.
 */
esp_err_t can_deinit(void);
