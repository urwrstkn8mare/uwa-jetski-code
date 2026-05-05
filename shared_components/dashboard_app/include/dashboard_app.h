#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "dashboard_ui.h"
#include "esp_err.h"
#include "lvgl.h"

typedef struct {
  dashboard_ui_t *ui;
  uint32_t start_ms;
  lv_obj_t *can_strip;
  lv_timer_t *timer;
} dashboard_app_t;

typedef struct {
  lv_obj_t *screen;
  int32_t h_res;
  int32_t v_res;
  int32_t can_strip_h;
  font_get_cb_t font_get_cb;
  void *font_user_data;
  bool enable_can_strip;
} dashboard_app_config_t;

esp_err_t dashboard_app_start(dashboard_app_t *app, const dashboard_app_config_t *cfg);
