#pragma once

#include "dashboard_ui.h"

#include <stddef.h>

/* Smooth animated telemetry for widget bring-up tests. */
void dashboard_demo_fill(dashboard_data_t *data, uint32_t elapsed_ms);

/** Feed callback wrapper (user is unused). */
void dashboard_demo_feed_cb(dashboard_data_t *out, uint32_t elapsed_ms, void *user);
