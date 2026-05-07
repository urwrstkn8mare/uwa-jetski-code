#include "status_ui.h"

#include "esp_log.h"
#include "esp_timer.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "status_ui";

esp_err_t status_ui_start(status_ui_t *disp,
                          const status_ui_cfg_t *cfg) {
  if (disp == NULL || cfg == NULL || cfg->parent == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  memset(disp, 0, sizeof(*disp));
  disp->lock_cb = cfg->lock_cb;
  disp->unlock_cb = cfg->unlock_cb;
  disp->min_interval_ms = cfg->min_interval_ms;

  disp->container = lv_obj_create(cfg->parent);
  lv_obj_remove_style_all(disp->container);
  lv_obj_set_style_border_width(disp->container, 0, 0);

  lv_opa_t opa = cfg->bg_opa ? cfg->bg_opa : LV_OPA_TRANSP;
  lv_obj_set_style_bg_color(disp->container, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(disp->container, opa, 0);

  lv_flex_flow_t flow = cfg->flex_flow ? cfg->flex_flow : LV_FLEX_FLOW_COLUMN;
  lv_obj_set_flex_flow(disp->container, flow);

  int32_t w = cfg->w ? cfg->w : LV_PCT(100);
  int32_t h = cfg->h ? cfg->h : LV_SIZE_CONTENT;
  lv_obj_set_size(disp->container, w, h);

  lv_obj_set_style_pad_all(disp->container, 4, 0);

  if (cfg->align != 0) {
    lv_obj_align(disp->container, cfg->align, 0, 0);
  }

  return ESP_OK;
}

void status_ui_stop(status_ui_t *disp) {
  if (disp == NULL) {
    return;
  }
  for (size_t i = 0; i < disp->entry_count; i++) {
    if (disp->lock_cb) {
      disp->lock_cb();
    }
    if (disp->entries[i].label != NULL) {
      lv_obj_delete(disp->entries[i].label);
      disp->entries[i].label = NULL;
    }
    if (disp->unlock_cb) {
      disp->unlock_cb();
    }
  }
  lv_obj_delete(disp->container);
  disp->container = NULL;
  memset(disp, 0, sizeof(*disp));
}

void status_ui_update(void *ctx, const char *tag,
                      const char *fmt, ...) {
  status_ui_t *disp = (status_ui_t *)ctx;
  if (disp == NULL || tag == NULL || fmt == NULL) {
    return;
  }

  const int64_t now = esp_timer_get_time();
  size_t idx = disp->entry_count;

  for (size_t i = 0; i < disp->entry_count; i++) {
    if (strncmp(disp->entries[i].tag, tag, sizeof(disp->entries[i].tag)) == 0) {
      idx = i;
      break;
    }
  }

  if (idx == disp->entry_count && idx >= (sizeof(disp->entries) / sizeof(disp->entries[0]))) {
    ESP_LOGW(TAG, "Too many status tags, ignoring \"%s\"", tag);
    return;
  }

  if (idx < disp->entry_count) {
    const int64_t min_dt = (int64_t)disp->min_interval_ms * 1000LL;
    if (disp->entries[idx].last_update_us > 0 && (now - disp->entries[idx].last_update_us) < min_dt) {
      return;
    }
  }

  int n = 0;
  {
    va_list args;
    va_start(args, fmt);
    n = vsnprintf(disp->scratch, sizeof(disp->scratch), fmt, args);
    va_end(args);
  }
  if (n < 0) {
    return;
  }
  if ((size_t)n >= sizeof(disp->scratch)) {
    n = (int)(sizeof(disp->scratch) - 1);
  }

  if (idx >= disp->entry_count) {
    if (disp->lock_cb) {
      disp->lock_cb();
    }
    lv_obj_t *label = lv_label_create(disp->container);
    if (label == NULL) {
      if (disp->unlock_cb) {
        disp->unlock_cb();
      }
      return;
    }
    strncpy(disp->entries[idx].tag, tag, sizeof(disp->entries[idx].tag) - 1);
    disp->entries[idx].label = label;
    disp->entries[idx].last_update_us = now;
    disp->entry_count++;
    lv_label_set_text(label, disp->scratch);
    if (disp->unlock_cb) {
      disp->unlock_cb();
    }
    return;
  }

  if (disp->lock_cb) {
    disp->lock_cb();
  }
  disp->entries[idx].last_update_us = now;
  lv_label_set_text(disp->entries[idx].label, disp->scratch);
  if (disp->unlock_cb) {
    disp->unlock_cb();
  }
}
