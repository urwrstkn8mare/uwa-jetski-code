#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * UTC wall clock for a device with no RTC battery and no internet. The clock
 * is an offset against the boot-time monotonic counter, set from the aux
 * controller's GPS over CAN or manually from a browser via the webui.
 */

typedef enum {
    WALLTIME_SOURCE_NONE = 0,
    WALLTIME_SOURCE_BROWSER,
    WALLTIME_SOURCE_GPS,
} walltime_source_t;

/* Start listening for GPS time frames on CAN. */
esp_err_t walltime_init(void);

void walltime_set(uint32_t epoch_s, walltime_source_t source);

/* Current UTC epoch seconds. Returns false while no source has set the
 * clock; either out pointer may be NULL. */
bool walltime_get(uint32_t *epoch_s, walltime_source_t *source);

/* Epoch seconds corresponding to a past moment given as ms since boot;
 * 0 while the clock is unset. */
uint32_t walltime_epoch_at_uptime_ms(int64_t uptime_ms);
