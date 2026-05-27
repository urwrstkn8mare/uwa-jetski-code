#include "status_ui.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "lvgl.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "status_ui";

#define MAX_ENTRIES     16
#define TAG_LEN         16
#define TEXT_CAP        256

typedef struct {
    char       tag[TAG_LEN];
    lv_obj_t  *row;
    lv_obj_t  *tag_label;
    lv_obj_t  *val_label;
    int64_t    last_update_us;
} entry_t;

typedef struct {
    lv_obj_t  *container;
    bool (*lock_cb)(uint32_t timeout_ms);
    void (*unlock_cb)(void);
    uint32_t   lock_timeout_ms;
    uint32_t   min_interval_ms;
    entry_t    entries[MAX_ENTRIES];
    size_t     entry_count;
    bool       initialized;
} status_ui_t;

static status_ui_t s_status_ui = {0};

esp_err_t status_ui_start(const status_ui_cfg_t *cfg) {
    if (cfg == NULL || cfg->parent == NULL) return ESP_ERR_INVALID_ARG;
    if (s_status_ui.initialized) {
        ESP_LOGW(TAG, "status_ui already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_status_ui, 0, sizeof(s_status_ui));
    s_status_ui.lock_cb         = cfg->lock_cb;
    s_status_ui.unlock_cb       = cfg->unlock_cb;
    s_status_ui.lock_timeout_ms = cfg->lock_timeout_ms;
    s_status_ui.min_interval_ms = cfg->min_interval_ms;

    if (s_status_ui.lock_cb) s_status_ui.lock_cb(s_status_ui.lock_timeout_ms);

    s_status_ui.container = lv_obj_create(cfg->parent);
    lv_obj_remove_style_all(s_status_ui.container);
    lv_obj_set_style_border_width(s_status_ui.container, 0, 0);
    lv_obj_set_style_bg_color(s_status_ui.container, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_status_ui.container,
                            cfg->bg_opa ? cfg->bg_opa : LV_OPA_TRANSP, 0);
    lv_obj_set_flex_flow(s_status_ui.container,
                         cfg->flex_flow ? cfg->flex_flow : LV_FLEX_FLOW_COLUMN);
    lv_obj_set_size(s_status_ui.container,
                    cfg->w ? cfg->w : LV_PCT(100),
                    cfg->h ? cfg->h : LV_PCT(100));
    lv_obj_set_style_pad_all(s_status_ui.container, 2, 0);

    /* Keep the status list static; only long values scroll horizontally. */
    lv_obj_clear_flag(s_status_ui.container, LV_OBJ_FLAG_SCROLLABLE);

    if (cfg->align != 0) lv_obj_align(s_status_ui.container, cfg->align, 0, 0);

    if (s_status_ui.unlock_cb) s_status_ui.unlock_cb();

    s_status_ui.initialized = true;
    return ESP_OK;
}

void status_ui_stop(void) {
    if (!s_status_ui.initialized) return;

    if (s_status_ui.lock_cb) s_status_ui.lock_cb(s_status_ui.lock_timeout_ms);

    for (size_t i = 0; i < s_status_ui.entry_count; i++) {
        if (s_status_ui.entries[i].row) {
            lv_obj_delete(s_status_ui.entries[i].row);
            s_status_ui.entries[i].row = NULL;
        }
    }

    if (s_status_ui.container) {
        lv_obj_delete(s_status_ui.container);
        s_status_ui.container = NULL;
    }

    if (s_status_ui.unlock_cb) s_status_ui.unlock_cb();
    memset(&s_status_ui, 0, sizeof(s_status_ui));
}

void status_ui_update(const char *tag, const char *fmt, ...) {
    if (!s_status_ui.initialized || tag == NULL || fmt == NULL) return;

    /* Format into a STACK-LOCAL buffer before acquiring the lock.
     * The old shared s_status_ui.scratch was formatted outside the lock,
     * so two concurrent callers would race and one would render the other's
     * text under its own tag. */
    char buf[TEXT_CAP];
    {
        va_list args;
        va_start(args, fmt);
        int n = vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);
        if (n < 0) return;
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
        if (s_status_ui.entries[idx].last_update_us > 0 &&
            (now - s_status_ui.entries[idx].last_update_us) < min_dt) {
            return;
        }
    }

    if (s_status_ui.lock_cb) s_status_ui.lock_cb(s_status_ui.lock_timeout_ms);

    if (idx >= s_status_ui.entry_count) {
        snprintf(s_status_ui.entries[idx].tag, TAG_LEN, "%s", tag);

        /* Row: flex-row container, full width, transparent background */
        lv_obj_t *row = lv_obj_create(s_status_ui.container);
        lv_obj_remove_style_all(row);
        lv_obj_set_width(row, LV_PCT(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_set_style_pad_column(row, 0, 0);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

        /* Tag label: cyan, clipped — never scrolls */
        lv_obj_t *tag_lbl = lv_label_create(row);
        lv_obj_set_style_text_color(tag_lbl, lv_color_hex(0x00E5FF), 0);
        lv_obj_set_style_text_opa(tag_lbl, LV_OPA_COVER, 0);
        lv_obj_set_height(tag_lbl, LV_SIZE_CONTENT);
        lv_label_set_long_mode(tag_lbl, LV_LABEL_LONG_MODE_CLIP);
        lv_label_set_text(tag_lbl, tag);

        /* Value label: fills remaining width, white, circular scroll */
        lv_obj_t *val_lbl = lv_label_create(row);
        lv_obj_set_style_text_color(val_lbl, lv_color_white(), 0);
        lv_obj_set_style_text_opa(val_lbl, LV_OPA_COVER, 0);
        lv_obj_set_flex_grow(val_lbl, 1);
        lv_obj_set_height(val_lbl, LV_SIZE_CONTENT);
        lv_label_set_long_mode(val_lbl, LV_LABEL_LONG_MODE_SCROLL_CIRCULAR);

        s_status_ui.entries[idx].row       = row;
        s_status_ui.entries[idx].tag_label = tag_lbl;
        s_status_ui.entries[idx].val_label = val_lbl;
        s_status_ui.entry_count++;

    }

    lv_label_set_text(s_status_ui.entries[idx].val_label, buf);
    s_status_ui.entries[idx].last_update_us = now;

    if (s_status_ui.unlock_cb) s_status_ui.unlock_cb();
}
