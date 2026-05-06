#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "lvgl.h"

typedef size_t (*lvgl_status_line_write_fn_t)(char *buf, size_t cap, void *ctx);

typedef struct {
  lvgl_status_line_write_fn_t write;
  void *ctx;
} lvgl_status_line_t;

typedef struct {
  lv_obj_t *label;
  lv_timer_t *timer;
  uint8_t *scratch;
  size_t scratch_cap;
  lvgl_status_line_t lines_storage[16];
  const lvgl_status_line_t *lines;
  size_t line_count;
} lvgl_status_display_t;

/**
 * Periodic debug text: concatenate each provider's UTF-8 line with newlines (`lines` lifetime must outlive stop).
 *
 * Caller must invoke from the LVGL task (timer runs on LVGL context).
 */
esp_err_t lvgl_status_display_start(lvgl_status_display_t *disp, lv_obj_t *label, const lvgl_status_line_t *lines,
                                     size_t line_count, uint32_t period_ms, size_t compose_cap);

void lvgl_status_display_stop(lvgl_status_display_t *disp);
