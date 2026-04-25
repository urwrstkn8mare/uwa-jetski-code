#include <stdbool.h>
#include <stdint.h>

// Return value is whether or not a higher priorirty task has been unblocked (for preemption)
typedef bool (*can_rx_cb_t)(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp);

/**
 * @brief Initialise CAN (twai)
 *
 * @param[in] can_rx_cb: Function to call when CAN frame received
 */
void can_init(can_rx_cb_t can_rx_cb);
