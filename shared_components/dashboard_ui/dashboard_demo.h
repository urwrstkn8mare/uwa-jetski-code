#pragma once

#include "dashboard_ui.h"

/* Generates smooth animated test data to exercise all dashboard widgets.
 * `elapsed_ms` is typically lv_tick_elaps(app_start_ms).
 * Replace this entire file when plumbing in real sensor/CAN data. */
void dashboard_demo_fill(dashboard_data_t *data, uint32_t elapsed_ms);
