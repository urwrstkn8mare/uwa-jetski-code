#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/*
 * Callback invoked when a CAN frame is received.
 *
 * Called from the CAN RX worker task context (never from ISR context).
 * Every registered callback is invoked for every received frame; each one
 * filters by header_id itself.
 */
typedef void (*can_rx_cb_t)(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp);

/**
 * @brief Initialise CAN (TWAI)
 *
 * Creates internal TX/RX worker tasks and starts the TWAI node.
 *
 * @return ESP_OK on success, error code on failure.
 */
esp_err_t can_init(void);

/**
 * @brief Register a callback to receive CAN frames.
 *
 * Callbacks may be registered before or after can_init(). Every registered
 * callback is invoked (in registration order) for every received frame.
 *
 * @param[in] cb Callback to register (must be non-NULL).
 * @return ESP_OK on success, ESP_ERR_NO_MEM if the callback table is full.
 */
esp_err_t can_register_rx_cb(can_rx_cb_t cb);

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

/* Per-ID receive statistics, for bus diagnostics. */
typedef struct {
    uint32_t id;           /* CAN identifier */
    uint32_t count;        /* frames received since boot */
    uint8_t  last_data[8]; /* payload of the most recent frame */
    uint8_t  last_len;     /* DLC of the most recent frame */
    int64_t  last_us;      /* esp_timer time of the most recent frame */
} can_rx_stat_t;

/**
 * @brief Snapshot per-ID RX statistics (every distinct ID seen since boot).
 *
 * @param[out] out Array to fill.
 * @param[in]  max Capacity of @p out.
 * @return Number of entries written.
 */
size_t can_get_rx_stats(can_rx_stat_t *out, size_t max);

/**
 * @brief Get TX outcome counters (frames acknowledged on the bus vs. dropped
 *        after retries, as reported by the driver's tx-done callback).
 */
void can_get_tx_stats(uint32_t *ok_out, uint32_t *fail_out);
