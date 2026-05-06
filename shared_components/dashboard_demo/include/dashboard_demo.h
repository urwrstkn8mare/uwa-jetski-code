#pragma once

#include "dashboard_ui.h"

/* Smooth animated telemetry for widget bring-up tests. */
void dashboard_demo_fill(dashboard_data_t *data, uint32_t elapsed_ms);

/* Generate demo telemetry and apply it to a dashboard UI. */
void dashboard_demo_update_ui(dashboard_ui_t *ui, uint32_t elapsed_ms);
