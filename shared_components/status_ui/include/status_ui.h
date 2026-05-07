#pragma once

#include "esp_err.h"
#include "lvgl.h"
#include <stdint.h>

typedef void (*status_write_cb_t)(const char *tag, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

typedef struct {
  lv_obj_t *parent;
  lv_flex_flow_t flex_flow;
  int32_t w;
  int32_t h;
  lv_align_t align;
  lv_opa_t bg_opa;
  void (*lock_cb)(void);
  void (*unlock_cb)(void);
  uint32_t min_interval_ms;
} status_ui_cfg_t;

/* Initialize the singleton status_ui instance.
 * Can only be called once; subsequent calls return ESP_ERR_INVALID_STATE.
 * Must be paired with status_ui_stop(). */
esp_err_t status_ui_start(const status_ui_cfg_t *cfg);

/* Clean up the singleton status_ui instance. */
void status_ui_stop(void);

/* Update status display with a formatted message for the given tag.
 * Creates new entry if tag doesn't exist, updates existing entry otherwise. */
void status_ui_update(const char *tag, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));
