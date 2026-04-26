#pragma once

#include <stdbool.h>
#include <stdint.h>

// Return value is whether or not a higher priority task has been unblocked (for preemption)
typedef bool (*can_rx_cb_t)(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp);

/**
 * @brief Initialise CAN (TWAI)
 *
 * @param[in] can_rx_cb: Function to call when CAN frame received. Can be NULL if RX not needed.
 */
void can_init(can_rx_cb_t can_rx_cb);

/**
 * @brief Transmit a CAN frame
 *
 * @param[in] id: CAN message identifier (11-bit standard ID)
 * @param[in] data: Pointer to data buffer (max 8 bytes)
 * @param[in] len: Data length (0-8)
 * @return true on success, false on failure
 */
bool can_tx(uint32_t id, const uint8_t *data, uint8_t len);
