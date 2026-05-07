#include "status_ui.h"

#include "esp_log.h"
#include "esp_timer.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "status_ui";

/* Singleton instance */
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
  bool initialized;
} status_ui_t;

static status_ui_t s_status_ui = {0};

esp_err_t status_ui_start(const status_ui_cfg_t *cfg) {
  if (cfg == NULL || cfg->parent == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (s_status_ui.initialized) {
    ESP_LOGW(TAG, "status_ui already initialized");
    return ESP_ERR_INVALID_STATE;
  }

  memset(&s_status_ui, 0, sizeof(s_status_ui));
  s_status_ui.lock_cb = cfg->lock_cb;
  s_status_ui.unlock_cb = cfg->unlock_cb;
  s_status_ui.min_interval_ms = cfg->min_interval_ms;

  /* Acquire lock before creating LVGL objects */
  if (s_status_ui.lock_cb) {
    s_status_ui.lock_cb();
  }

  s_status_ui.container = lv_obj_create(cfg->parent);
  lv_obj_remove_style_all(s_status_ui.container);
  lv_obj_set_style_border_width(s_status_ui.container, 0, 0);

  lv_opa_t opa = cfg->bg_opa ? cfg->bg_opa : LV_OPA_TRANSP;
  lv_obj_set_style_bg_color(s_status_ui.container, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(s_status_ui.container, opa, 0);

  lv_flex_flow_t flow = cfg->flex_flow ? cfg->flex_flow : LV_FLEX_FLOW_COLUMN;
  lv_obj_set_flex_flow(s_status_ui.container, flow);

  int32_t w = cfg->w ? cfg->w : LV_PCT(100);
  int32_t h = cfg->h ? cfg->h : LV_SIZE_CONTENT;
  lv_obj_set_size(s_status_ui.container, w, h);

  lv_obj_set_style_pad_all(s_status_ui.container, 4, 0);

  if (cfg->align != 0) {
    lv_obj_align(s_status_ui.container, cfg->align, 0, 0);
  }

  /* Release lock after creating LVGL objects */
  if (s_status_ui.unlock_cb) {
    s_status_ui.unlock_cb();
  }

  s_status_ui.initialized = true;
  return ESP_OK;
}

void status_ui_stop(void) {
  if (!s_status_ui.initialized) {
    return;
  }

  for (size_t i = 0; i < s_status_ui.entry_count; i++) {
    if (s_status_ui.lock_cb) {
      s_status_ui.lock_cb();
    }
    if (s_status_ui.entries[i].label != NULL) {
      lv_obj_delete(s_status_ui.entries[i].label);
      s_status_ui.entries[i].label = NULL;
    }
    if (s_status_ui.unlock_cb) {
      s_status_ui.unlock_cb();
    }
  }

  /* Acquire lock before deleting container */
  if (s_status_ui.lock_cb) {
    s_status_ui.lock_cb();
  }
  lv_obj_delete(s_status_ui.container);
  if (s_status_ui.unlock_cb) {
    s_status_ui.unlock_cb();
  }

  s_status_ui.container = NULL;
  memset(&s_status_ui, 0, sizeof(s_status_ui));
}

void status_ui_update(const char *tag, const char *fmt, ...) {
  if (!s_status_ui.initialized) {
    return;
  }
  if (tag == NULL || fmt == NULL) {
    return;
  }

  const int64_t now = esp_timer_get_time();
  size_t idx = s_status_ui.entry_count;

  for (size_t i = 0; i < s_status_ui.entry_count; i++) {
    if (strncmp(s_status_ui.entries[i].tag, tag, sizeof(s_status_ui.entries[i].tag)) == 0) {
      idx = i;
      break;
    }
  }

  if (idx == s_status_ui.entry_count && idx >= (sizeof(s_status_ui.entries) / sizeof(s_status_ui.entries[0]))) {
    ESP_LOGW(TAG, "Too many status tags, ignoring \"%s\"", tag);
    return;
  }

  if (idx < s_status_ui.entry_count) {
    const int64_t min_dt = (int64_t)s_status_ui.min_interval_ms * 1000LL;
    if (s_status_ui.entries[idx].last_update_us > 0 && (now - s_status_ui.entries[idx].last_update_us) < min_dt) {
      return;
    }
  }

  int n = 0;
  {
    va_list args;
    va_start(args, fmt);
    n = vsnprintf(s_status_ui.scratch, sizeof(s_status_ui.scratch), fmt, args);
    va_end(args);
  }
  if (n < 0) {
    return;
  }
  if ((size_t)n >= sizeof(s_status_ui.scratch)) {
    n = (int)(sizeof(s_status_ui.scratch) - 1);
  }

  if (idx >= s_status_ui.entry_count) {
    if (s_status_ui.lock_cb) {
      s_status_ui.lock_cb();
    }
    lv_obj_t *label = lv_label_create(s_status_ui.container);
    if (label == NULL) {
      if (s_status_ui.unlock_cb) {
        s_status_ui.unlock_cb();
      }
      return;
    }
    strncpy(s_status_ui.entries[idx].tag, tag, sizeof(s_status_ui.entries[idx].tag) - 1);
    s_status_ui.entries[idx].label = label;
    s_status_ui.entries[idx].last_update_us = now;
    s_status_ui.entry_count++;
    lv_label_set_text(label, s_status_ui.scratch);
    if (s_status_ui.unlock_cb) {
      s_status_ui.unlock_cb();
    }
    return;
  }

  if (s_status_ui.lock_cb) {
    s_status_ui.lock_cb();
  }
  s_status_ui.entries[idx].last_update_us = now;
  lv_label_set_text(s_status_ui.entries[idx].label, s_status_ui.scratch);
  if (s_status_ui.unlock_cb) {
    s_status_ui.unlock_cb();
  }
}
