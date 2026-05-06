#pragma once

#include <stddef.h>
#include <stdint.h>

#include "dashboard_ui.h"
#include "esp_err.h"
#include "lvgl.h"

typedef esp_err_t (*dashboard_can_lock_fn_t)(int32_t timeout_ms, void *ctx);
typedef void (*dashboard_can_unlock_fn_t)(void *ctx);

/**
 * Attach the CAN feed to a dashboard UI.
 *
 * The CAN RX callback runs on the `can_rx` worker task (not ISR). This module
 * will take the provided display lock briefly to apply surgical UI updates.
 */
esp_err_t dashboard_can_attach(dashboard_ui_t *ui, dashboard_can_lock_fn_t lock, dashboard_can_unlock_fn_t unlock,
                               void *lock_ctx);

/** Start TWAI and register the RX callback. */
esp_err_t dashboard_can_start(void);

/** One-line status strip text. */
size_t dashboard_can_status_strip_write(char *buffer, size_t len, void *user);

/** Fallback status strip text when CAN is not available. */
size_t dashboard_can_unavailable_status_strip_write(char *buffer, size_t len, void *user);
