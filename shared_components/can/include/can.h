#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/*
 * Callback invoked when a CAN frame is received.
 *
 * Called from the CAN RX worker task context (never from ISR context).
 */
typedef void (*can_rx_cb_t)(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp);

/**
 * @brief Initialise CAN (TWAI)
 *
 * Creates internal TX/RX worker tasks and starts the TWAI node.
 *
 * @param[in] can_rx_cb Function to call when CAN frame received. Can be NULL.
 * @return ESP_OK on success, error code on failure.
 */
esp_err_t can_init(can_rx_cb_t can_rx_cb);

/**
 * @brief Transmit a CAN frame
 *
 * @param[in] id: CAN message identifier (11-bit standard ID)
 * @param[in] data: Pointer to data buffer (max 8 bytes)
 * @param[in] len: Data length (0-8)
 * @return true on success, false on failure
 */
bool can_tx(uint32_t id, const uint8_t *data, uint8_t len);

/**
 * @brief Get CAN TX diagnostics (attempts and failures)
 *
 * @param[out] attempts: Total transmission attempts
 * @param[out] failures: Total transmission failures
 */
void can_get_tx_stats(uint32_t *attempts, uint32_t *failures);

/**
 * @brief Deinitialise CAN (TWAI)
 *
 * Stops the TWAI node, deletes worker tasks and frees queues.
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not initialised.
 */
esp_err_t can_deinit(void);
