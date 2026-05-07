#pragma once

#include "esp_err.h"
#include "lvgl.h"
#include <stdint.h>

typedef void (*status_write_cb_t)(void *ctx, const char *tag, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

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

typedef struct {
  lv_obj_t *container;
  void (*lock_cb)(void);
  void (*unlock_cb)(void);
  uint32_t min_interval_ms;
  struct {
    char tag[16];
    lv_obj_t *label;
    int64_t last_update_us;
  } entries[16];
  size_t entry_count;
  char scratch[256];
} status_ui_t;

esp_err_t status_ui_start(status_ui_t *disp,
                          const status_ui_cfg_t *cfg);

void status_ui_stop(status_ui_t *disp);

void status_ui_update(void *ctx, const char *tag,
                      const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));
