#pragma once

#include <stddef.h>

#include "esp_err.h"

/* Generic NVS-backed key/value blob store.
 *
 * config knows nothing about what it stores — each component owns its own
 * config struct and its defaults, and persists it here under a unique key. */

esp_err_t config_init(void);

/* Read a blob into out. Returns ESP_OK only if a blob of exactly `size` bytes
 * exists under `key`; any other result (missing key, size mismatch) is an error
 * and the caller should fall back to defaults. */
esp_err_t config_get_blob(const char *key, void *out, size_t size);

/* Write a blob under `key` and commit. */
esp_err_t config_set_blob(const char *key, const void *data, size_t size);
