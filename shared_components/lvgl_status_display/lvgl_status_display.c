#include "lvgl_status_display.h"

#include "esp_log.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "lvgl_status_dsp";

static void refresh_cb(lv_timer_t *timer) {
  lvgl_status_display_t *disp = lv_timer_get_user_data(timer);
  if (disp == NULL || disp->label == NULL || disp->scratch == NULL || disp->line_count == 0) {
    return;
  }
  uint8_t *cursor = disp->scratch;
  size_t rem = disp->scratch_cap;
  cursor[0] = '\0';
  bool appended = false;
  for (size_t i = 0; i < disp->line_count && rem > 1; i++) {
    if (disp->lines == NULL || disp->lines[i].write == NULL) {
      continue;
    }
    if (appended && rem > 1) {
      *cursor++ = '\n';
      rem--;
      *cursor = '\0';
    }
    size_t nw = disp->lines[i].write((char *)cursor, rem, disp->lines[i].ctx);
    if (nw >= rem) {
      nw = rem - 1;
    }
    cursor += nw;
    rem -= nw;
    *cursor = '\0';
    appended = true;
  }
  lv_label_set_text(disp->label, (const char *)disp->scratch);
}

esp_err_t lvgl_status_display_start(lvgl_status_display_t *disp, lv_obj_t *label, const lvgl_status_line_t *lines,
                                     size_t line_count, uint32_t period_ms, size_t compose_cap) {
  if (disp == NULL || label == NULL || lines == NULL || line_count == 0 || compose_cap == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  if (line_count > sizeof(disp->lines_storage) / sizeof(disp->lines_storage[0])) {
    ESP_LOGE(TAG, "Too many lines (max %u)", (unsigned)(sizeof(disp->lines_storage) / sizeof(disp->lines_storage[0])));
    return ESP_ERR_INVALID_ARG;
  }

  memset(disp, 0, sizeof(*disp));
  disp->scratch = (uint8_t *)malloc(compose_cap);
  if (disp->scratch == NULL) {
    return ESP_ERR_NO_MEM;
  }
  disp->scratch_cap = compose_cap;
  memcpy(disp->lines_storage, lines, line_count * sizeof(lvgl_status_line_t));
  disp->lines = disp->lines_storage;
  disp->line_count = line_count;
  disp->label = label;

  disp->timer = lv_timer_create(refresh_cb, period_ms, disp);
  if (disp->timer == NULL) {
    free(disp->scratch);
    disp->scratch = NULL;
    return ESP_ERR_NO_MEM;
  }
  return ESP_OK;
}

void lvgl_status_display_stop(lvgl_status_display_t *disp) {
  if (disp == NULL) {
    return;
  }
  if (disp->timer != NULL) {
    lv_timer_delete(disp->timer);
    disp->timer = NULL;
  }
  if (disp->scratch != NULL) {
    free(disp->scratch);
    disp->scratch = NULL;
  }
  memset(disp, 0, sizeof(*disp));
}
