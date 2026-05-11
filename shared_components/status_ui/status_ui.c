#include "status_ui.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "widgets/label/lv_label.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "status_ui";

enum { MAX_ENTRIES = 16 };
enum { TAG_LEN = 16 };
enum { TEXT_CAP = 256 };

typedef struct {
  char tag[TAG_LEN];
  char text[TEXT_CAP];
  lv_obj_t *label;
  int64_t last_update_us;
} entry_t;

typedef struct {
  lv_obj_t *container;
  void (*lock_cb)(void);
  void (*unlock_cb)(void);
  uint32_t min_interval_ms;
  entry_t entries[MAX_ENTRIES];
  size_t entry_count;
  char scratch[TEXT_CAP];
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

  if (s_status_ui.lock_cb) {
    s_status_ui.lock_cb();
  }

  for (size_t i = 0; i < s_status_ui.entry_count; i++) {
    if (s_status_ui.entries[i].label != NULL) {
      lv_obj_delete(s_status_ui.entries[i].label);
      s_status_ui.entries[i].label = NULL;
    }
  }

  if (s_status_ui.container != NULL) {
    lv_obj_delete(s_status_ui.container);
    s_status_ui.container = NULL;
  }

  if (s_status_ui.unlock_cb) {
    s_status_ui.unlock_cb();
  }

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
    if (strncmp(s_status_ui.entries[i].tag, tag, TAG_LEN) == 0) {
      idx = i;
      break;
    }
  }

  if (idx == s_status_ui.entry_count && idx >= MAX_ENTRIES) {
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

  if (s_status_ui.lock_cb) {
    s_status_ui.lock_cb();
  }

  if (idx >= s_status_ui.entry_count) {
    snprintf(s_status_ui.entries[idx].tag, TAG_LEN, "%s", tag);
    lv_obj_t *lbl = lv_label_create(s_status_ui.container);
    lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
    lv_obj_set_style_text_opa(lbl, LV_OPA_COVER, 0);
    lv_label_set_recolor(lbl, true);
    lv_label_set_long_mode(lbl, LV_LABEL_LONG_MODE_SCROLL_CIRCULAR);
    s_status_ui.entries[idx].label = lbl;
    s_status_ui.entry_count++;
  }

  char *text = s_status_ui.entries[idx].text;
  char tag_copy[TAG_LEN];
  memcpy(tag_copy, s_status_ui.entries[idx].tag, TAG_LEN);
  int written = snprintf(text, TEXT_CAP, "#00E5FF %s# %s", tag_copy, s_status_ui.scratch);
  if (written < 0) {
    written = 0;
  }
  if ((size_t)written >= TEXT_CAP) {
    text[TEXT_CAP - 1] = '\0';
  }

  lv_label_set_text(s_status_ui.entries[idx].label, text);
  s_status_ui.entries[idx].last_update_us = now;

  if (s_status_ui.unlock_cb) {
    s_status_ui.unlock_cb();
  }
}
