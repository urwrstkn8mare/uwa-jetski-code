#include "dashboard_ui.h"

#include "lvgl.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

LV_FONT_DECLARE(lv_font_montserrat_14);
LV_FONT_DECLARE(lv_font_montserrat_18);
LV_FONT_DECLARE(lv_font_montserrat_48);

static const char *TAG = "dashboard_ui";

static const lv_font_t *speed_font = &lv_font_montserrat_48;
static const lv_font_t *height_value_font = &lv_font_montserrat_48;
static const lv_font_t *control_font = &lv_font_montserrat_18;
static const lv_font_t *percent_font = &lv_font_montserrat_18;
static const lv_font_t *elevon_font = &lv_font_montserrat_18;
static const lv_font_t *small_font = &lv_font_montserrat_14;
static const lv_font_t *title_font = &lv_font_montserrat_14;

typedef enum {
  kUiSpeed = 0,
  kUiHeight,
  kUiAttitude,
  kUiBattery,
  kUiMotor0,
  kUiMotor1,
  kUiRudder,
  kUiElevons,
  kUiThrottleCount,
} ui_throttle_t;

static bool ui_throttle_ok(dashboard_ui_t *ui, ui_throttle_t t);

enum {
  HEIGHT_TRACK_X = 54,
  HEIGHT_TRACK_Y = 38,
  HEIGHT_TRACK_W = 4,
  HEIGHT_TRACK_H = 196,
  HEIGHT_MIN_CM = 0,
  HEIGHT_MAX_CM = 50,
};

enum {
  ATTITUDE_SCALE_PX = 6,
  ATTITUDE_TICK_COUNT = 6,
};

typedef struct {
  lv_obj_t *root;
  lv_obj_t *value_label;
  lv_obj_t *unit_label;
  lv_obj_t *caption_label;
  lv_obj_t *bar;
  int32_t card_w;
  int32_t card_h;
} speed_card_t;

typedef struct {
  lv_obj_t *root;
  lv_obj_t *current_label;
  lv_obj_t *target_label;
  lv_obj_t *top_label;
  lv_obj_t *bottom_label;
  lv_obj_t *track;
  lv_obj_t *current_marker;
  lv_obj_t *target_marker;
  int32_t card_w;
  int32_t card_h;
} height_card_t;

typedef struct {
  lv_obj_t *root;
  lv_obj_t *canvas;
  lv_obj_t *heading_label;
  lv_obj_t *roll_label;
  lv_obj_t *pitch_label;
  lv_obj_t *pitch_marks[ATTITUDE_TICK_COUNT];
  void *draw_buf_mem;
  int32_t roll_deg;
  int32_t pitch_deg;
  int32_t heading_deg;
  int32_t card_w;
  int32_t card_h;
} attitude_card_t;

typedef struct {
  lv_obj_t *root;
  lv_obj_t *percent_label;
  lv_obj_t *voltage_label;
  lv_obj_t *current_label;
  lv_obj_t *temp_label;
  lv_obj_t *bar;
  int32_t card_w;
  int32_t card_h;
} battery_card_t;

typedef struct {
  lv_obj_t *root;
  lv_obj_t *percent_label;
  lv_obj_t *power_label;
  lv_obj_t *rpm_label;
  lv_obj_t *temp_label;
  lv_obj_t *bar;
  int32_t card_w;
  int32_t card_h;
} motor_card_t;

typedef struct {
  lv_obj_t *root;
  lv_obj_t *value_label;
  lv_obj_t *bar;
  lv_obj_t *zero_mark;
  int32_t card_w;
  int32_t card_h;
} control_surface_card_t;

struct dashboard_ui {
  lv_obj_t *root;
  speed_card_t speed;
  height_card_t height;
  attitude_card_t attitude;
  battery_card_t battery;
  motor_card_t motors[DASHBOARD_MOTOR_MAX];
  control_surface_card_t rudder;
  control_surface_card_t elevon_left;
  control_surface_card_t elevon_right;
  int32_t h_res;
  int32_t v_res;
  int64_t last_update_us[kUiThrottleCount];
};

static bool ui_throttle_ok(dashboard_ui_t *ui, ui_throttle_t t) {
  const int hz = (CONFIG_DASHBOARD_UI_THROTTLE_HZ > 0) ? CONFIG_DASHBOARD_UI_THROTTLE_HZ : 1;
  const int64_t min_dt = 1000000LL / (int64_t)hz;
  const int64_t now = esp_timer_get_time();
  if (ui == NULL) {
    return false;
  }
  if (ui->last_update_us[t] == 0 || (now - ui->last_update_us[t]) >= min_dt) {
    ui->last_update_us[t] = now;
    return true;
  }
  return false;
}

static const int32_t s_attitude_ticks[ATTITUDE_TICK_COUNT] = {30, 20, 10, -10,
                                                              -20, -30};

static int32_t clamp_i32(int32_t value, int32_t min, int32_t max) {
  if (value < min) {
    return min;
  }
  if (value > max) {
    return max;
  }
  return value;
}

static int32_t normalize_heading(int32_t heading_deg) {
  heading_deg %= 360;
  if (heading_deg < 0) {
    heading_deg += 360;
  }
  return heading_deg;
}

static int32_t get_baseline_offset(const lv_font_t *base_font,
                                   const lv_font_t *label_font) {
  if (base_font == NULL || label_font == NULL) {
    return 0;
  }

  return (base_font->line_height - base_font->base_line) -
         (label_font->line_height - label_font->base_line);
}

static const lv_font_t *load_font_variant(uint16_t size_px, int weight,
                                          font_get_cb_t font_get_cb,
                                          void *font_get_user_data,
                                          const lv_font_t *fallback) {
  if (font_get_cb != NULL) {
    const lv_font_t *font = font_get_cb(size_px, weight, font_get_user_data);
    if (font != NULL) {
      return font;
    }
  }
  return fallback;
}

static void init_dashboard_fonts(font_get_cb_t font_get_cb,
                                 void *font_get_user_data) {
  speed_font = load_font_variant(96, 700, font_get_cb, font_get_user_data,
                                 &lv_font_montserrat_48);
  height_value_font = load_font_variant(56, 700, font_get_cb,
                                        font_get_user_data,
                                        &lv_font_montserrat_48);
  control_font = load_font_variant(18, 700, font_get_cb, font_get_user_data,
                                   &lv_font_montserrat_18);
  percent_font = load_font_variant(24, 700, font_get_cb, font_get_user_data,
                                   &lv_font_montserrat_18);
  elevon_font = load_font_variant(14, 700, font_get_cb, font_get_user_data,
                                  &lv_font_montserrat_18);
  small_font = load_font_variant(14, 700, font_get_cb, font_get_user_data,
                                 &lv_font_montserrat_14);
  title_font = load_font_variant(14, 700, font_get_cb, font_get_user_data,
                                 &lv_font_montserrat_14);
}

static void configure_screen(lv_obj_t *screen) {
  if (screen == NULL) {
    return;
  }

  lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
  lv_obj_set_style_pad_all(screen, 0, 0);
  lv_obj_set_style_border_width(screen, 0, 0);
  lv_obj_set_style_outline_width(screen, 0, 0);
  lv_obj_set_style_radius(screen, 0, 0);
  lv_obj_set_scrollbar_mode(screen, LV_SCROLLBAR_MODE_OFF);
}

static lv_obj_t *create_dashboard_root(lv_obj_t *screen, int32_t h_res, int32_t v_res) {
  lv_obj_t *root = lv_obj_create(screen);
  if (root == NULL) {
    return NULL;
  }

  lv_obj_remove_style_all(root);
  lv_obj_remove_flag(root, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_pos(root, 0, 0);
  lv_obj_set_size(root, h_res, v_res);
  lv_obj_set_style_bg_opa(root, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(root, 0, 0);
  lv_obj_set_style_outline_width(root, 0, 0);
  lv_obj_set_style_radius(root, 0, 0);
  lv_obj_set_style_pad_all(root, 0, 0);
  lv_obj_set_style_pad_row(root, -1, 0);
  lv_obj_set_style_pad_column(root, -1, 0);
  lv_obj_set_flex_flow(root, LV_FLEX_FLOW_ROW_WRAP);
  lv_obj_set_flex_align(root, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START,
                        LV_FLEX_ALIGN_START);
  lv_obj_set_scrollbar_mode(root, LV_SCROLLBAR_MODE_OFF);
  return root;
}

static lv_obj_t *create_panel(lv_obj_t *parent, int32_t w, int32_t h,
                              lv_color_t bg) {
  lv_obj_t *panel = lv_obj_create(parent);
  if (panel == NULL) {
    return NULL;
  }

  lv_obj_remove_style_all(panel);
  lv_obj_remove_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(panel, w, h);
  lv_obj_set_style_bg_color(panel, bg, 0);
  lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(panel, 1, 0);
  lv_obj_set_style_border_color(panel, lv_color_hex(0x4B4B4B), 0);
  lv_obj_set_style_border_opa(panel, (lv_opa_t)(255 * 0.85), 0);
  lv_obj_set_style_radius(panel, 0, 0);
  lv_obj_set_style_pad_all(panel, 0, 0);
  return panel;
}

static lv_obj_t *add_title(lv_obj_t *panel, const char *text) {
  lv_obj_t *title = lv_label_create(panel);
  if (title == NULL) {
    return NULL;
  }

  lv_label_set_text(title, text);
  lv_obj_set_style_text_font(title, title_font, 0);
  lv_obj_set_style_text_color(title, lv_color_white(), 0);
  lv_obj_set_style_text_opa(title, (lv_opa_t)(255 * 0.50), 0);
  lv_label_set_long_mode(title, LV_LABEL_LONG_MODE_CLIP);
  lv_obj_add_flag(title, LV_OBJ_FLAG_IGNORE_LAYOUT);
  lv_obj_align(title, LV_ALIGN_TOP_RIGHT, -8, 5);
  return title;
}

static lv_obj_t *add_simple_label(lv_obj_t *parent, const char *text,
                                  const lv_font_t *font, lv_color_t color,
                                  lv_opa_t opa) {
  lv_obj_t *label = lv_label_create(parent);
  if (label == NULL) {
    return NULL;
  }

  lv_label_set_text(label, text);
  lv_obj_set_style_text_font(label, font, 0);
  lv_obj_set_style_text_color(label, color, 0);
  lv_obj_set_style_text_opa(label, opa, 0);
  lv_label_set_long_mode(label, LV_LABEL_LONG_MODE_CLIP);
  return label;
}

static lv_obj_t *create_pitch_label(lv_obj_t *parent, const char *text) {
  lv_obj_t *label = lv_label_create(parent);
  if (label == NULL) {
    return NULL;
  }

  lv_label_set_text(label, text);
  lv_obj_set_style_text_font(label, small_font, 0);
  lv_obj_set_style_text_color(label, lv_color_white(), 0);
  lv_obj_set_style_text_opa(label, (lv_opa_t)(255 * 0.85), 0);
  lv_label_set_long_mode(label, LV_LABEL_LONG_MODE_CLIP);
  lv_obj_add_flag(label, LV_OBJ_FLAG_IGNORE_LAYOUT);
  lv_obj_update_layout(label);
  lv_obj_set_style_transform_pivot_x(label, lv_obj_get_width(label) / 2, 0);
  lv_obj_set_style_transform_pivot_y(label, lv_obj_get_height(label) / 2, 0);
  return label;
}

static lv_obj_t *create_metrics_box(lv_obj_t *parent) {
  lv_obj_t *box = lv_obj_create(parent);
  if (box == NULL) {
    return NULL;
  }

  lv_obj_remove_style_all(box);
  lv_obj_set_size(box, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(box, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(box, 0, 0);
  lv_obj_set_style_border_opa(box, LV_OPA_TRANSP, 0);
  lv_obj_set_style_pad_all(box, 0, 0);
  lv_obj_set_style_pad_row(box, 0, 0);
  lv_obj_set_flex_flow(box, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(box, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER);
  lv_obj_add_flag(box, LV_OBJ_FLAG_IGNORE_LAYOUT);
  lv_obj_align(box, LV_ALIGN_CENTER, 0, 10);
  return box;
}

static lv_color_t percent_to_color(int32_t percent) {
  if (percent < 25) {
    return lv_color_hex(0xFF5A52);
  }
  if (percent < 70) {
    return lv_color_hex(0xFFD23F);
  }
  return lv_color_hex(0x4BE37C);
}

static void apply_percent_color(lv_obj_t *bar, lv_obj_t *label, int32_t percent) {
  const lv_color_t color = percent_to_color(percent);
  if (bar != NULL) {
    lv_obj_set_style_bg_color(bar, color, LV_PART_INDICATOR);
  }
  if (label != NULL) {
    lv_obj_set_style_text_color(label, color, 0);
  }
}

static lv_obj_t *create_bar(lv_obj_t *parent, int32_t w, int32_t h,
                            lv_bar_mode_t mode,
                            lv_bar_orientation_t orientation, int32_t min,
                            int32_t max, lv_color_t indicator_color,
                            lv_opa_t indicator_opa) {
  lv_obj_t *bar = lv_bar_create(parent);
  if (bar == NULL) {
    return NULL;
  }

  lv_obj_remove_style_all(bar);
  lv_obj_set_size(bar, w, h);
  lv_obj_set_style_radius(bar, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(bar, 0, LV_PART_INDICATOR);
  lv_obj_set_style_border_width(bar, 0, LV_PART_MAIN);
  lv_obj_set_style_border_width(bar, 0, LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(bar, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(bar, indicator_opa, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(bar, indicator_color, LV_PART_INDICATOR);
  lv_bar_set_mode(bar, mode);
  lv_bar_set_orientation(bar, orientation);
  lv_bar_set_range(bar, min, max);
  return bar;
}

static int32_t tick_half_len_for_pitch(int32_t pitch_deg) {
  LV_UNUSED(pitch_deg);
  return 50;
}

static void attitude_project_point(int32_t cx, int32_t cy, int32_t x,
                                   int32_t y, int32_t pitch_px,
                                   int32_t roll_deg, int32_t *rx,
                                   int32_t *ry) {
  const int32_t s = lv_trigo_sin((int16_t)roll_deg);
  const int32_t c = lv_trigo_cos((int16_t)roll_deg);
  const int32_t y_shifted = y + pitch_px;

  *rx = cx + (((x * c) - (y_shifted * s)) >> LV_TRIGO_SHIFT);
  *ry = cy + (((x * s) + (y_shifted * c)) >> LV_TRIGO_SHIFT);
}

static void attitude_canvas_fill_background(attitude_card_t *card) {
  if (card == NULL || card->canvas == NULL) {
    return;
  }

  lv_draw_buf_t *draw_buf = lv_canvas_get_draw_buf(card->canvas);
  if (draw_buf == NULL || draw_buf->header.cf != LV_COLOR_FORMAT_RGB565) {
    return;
  }

  const int32_t w = (int32_t)draw_buf->header.w;
  const int32_t h = (int32_t)draw_buf->header.h;
  const int32_t cx = w / 2;
  const int32_t cy = h / 2;
  const int32_t roll = clamp_i32(card->roll_deg, -45, 45);
  const int32_t pitch_px = clamp_i32(card->pitch_deg, -40, 40) * ATTITUDE_SCALE_PX;
  const uint16_t sky = lv_color_to_u16(lv_color_hex(0x2E93FF));
  const uint16_t ground = lv_color_to_u16(lv_color_hex(0x8C623A));
  int32_t hx1;
  int32_t hy1;
  int32_t hx2;
  int32_t hy2;

  attitude_project_point(cx, cy, -w, 0, pitch_px, roll, &hx1, &hy1);
  attitude_project_point(cx, cy, w, 0, pitch_px, roll, &hx2, &hy2);

  const int32_t dx = hx2 - hx1;
  const int32_t dy = hy2 - hy1;

  for (int32_t y = 0; y < h; y++) {
    uint16_t *row = (uint16_t *)lv_draw_buf_goto_xy(draw_buf, 0, (uint32_t)y);
    if (row == NULL) {
      continue;
    }

    int32_t side = ((0 - hx1) * dy) - ((y - hy1) * dx);
    for (int32_t x = 0; x < w; x++) {
      row[x] = (side > 0) ? sky : ground;
      side += dy;
    }
  }

  lv_draw_buf_flush_cache(draw_buf, NULL);
}

static void attitude_canvas_draw(attitude_card_t *card, lv_layer_t *layer) {
  if (card == NULL || layer == NULL) {
    return;
  }

  const int32_t w = card->card_w - 2;
  const int32_t h = card->card_h - 2;
  const int32_t cx = w / 2;
  const int32_t cy = h / 2;
  const int32_t roll = clamp_i32(card->roll_deg, -45, 45);
  const int32_t pitch_px = clamp_i32(card->pitch_deg, -40, 40) * ATTITUDE_SCALE_PX;

  lv_draw_line_dsc_t line;
  lv_draw_line_dsc_init(&line);
  line.color = lv_color_hex(0xF8F8F8);
  line.width = 2;
  line.round_start = 1;
  line.round_end = 1;

  attitude_project_point(cx, cy, -w, 0, pitch_px, roll, &line.p1.x,
                         &line.p1.y);
  attitude_project_point(cx, cy, w, 0, pitch_px, roll, &line.p2.x,
                         &line.p2.y);
  lv_draw_line(layer, &line);

  for (uint32_t i = 0; i < ATTITUDE_TICK_COUNT; i++) {
    const int32_t tick_pitch = s_attitude_ticks[i];
    const int32_t tick_y = -(tick_pitch * ATTITUDE_SCALE_PX);
    const int32_t len = tick_half_len_for_pitch(tick_pitch);
    const int32_t gap = 18;
    int32_t x1;
    int32_t y1;
    int32_t x2;
    int32_t y2;
    int32_t x3;
    int32_t y3;
    int32_t x4;
    int32_t y4;

    attitude_project_point(cx, cy, -len, tick_y, pitch_px, roll, &x1, &y1);
    attitude_project_point(cx, cy, -gap, tick_y, pitch_px, roll, &x2, &y2);
    attitude_project_point(cx, cy, gap, tick_y, pitch_px, roll, &x3, &y3);
    attitude_project_point(cx, cy, len, tick_y, pitch_px, roll, &x4, &y4);

    line.width = 2;
    line.opa = (lv_opa_t)(255 * 0.78);
    line.color = lv_color_hex(0xD8D8D8);
    line.p1.x = x1;
    line.p1.y = y1;
    line.p2.x = x2;
    line.p2.y = y2;
    lv_draw_line(layer, &line);

    line.p1.x = x3;
    line.p1.y = y3;
    line.p2.x = x4;
    line.p2.y = y4;
    lv_draw_line(layer, &line);
  }

  lv_draw_line_dsc_t center;
  lv_draw_line_dsc_init(&center);
  center.color = lv_color_hex(0xF1D400);
  center.width = 3;
  center.round_start = 1;
  center.round_end = 1;
  center.p1.x = cx - 14;
  center.p1.y = cy;
  center.p2.x = cx - 5;
  center.p2.y = cy;
  lv_draw_line(layer, &center);

  center.p1.x = cx - 5;
  center.p1.y = cy;
  center.p2.x = cx;
  center.p2.y = cy - 9;
  lv_draw_line(layer, &center);

  center.p1.x = cx;
  center.p1.y = cy - 9;
  center.p2.x = cx + 5;
  center.p2.y = cy;
  lv_draw_line(layer, &center);

  center.p1.x = cx + 5;
  center.p1.y = cy;
  center.p2.x = cx + 14;
  center.p2.y = cy;
  lv_draw_line(layer, &center);

  lv_draw_line_dsc_t wing;
  lv_draw_line_dsc_init(&wing);
  wing.color = lv_color_hex(0xF1D400);
  wing.width = 3;
  wing.round_start = 1;
  wing.round_end = 1;
  wing.p1.x = cx - 56;
  wing.p1.y = cy;
  wing.p2.x = cx - 14;
  wing.p2.y = cy;
  lv_draw_line(layer, &wing);
  wing.p1.x = cx + 14;
  wing.p2.x = cx + 56;
  lv_draw_line(layer, &wing);
}

static speed_card_t create_speed_card(lv_obj_t *parent, int32_t w, int32_t h) {
  speed_card_t card = {0};
  card.card_w = w;
  card.card_h = h;
  card.root = create_panel(parent, w, h, lv_color_black());
  if (card.root == NULL) {
    return card;
  }

  card.bar = lv_bar_create(card.root);
  lv_obj_remove_style_all(card.bar);
  lv_obj_add_flag(card.bar, LV_OBJ_FLAG_IGNORE_LAYOUT);
  lv_obj_set_pos(card.bar, 0, 0);
  lv_obj_set_size(card.bar, lv_pct(100), lv_pct(100));
  lv_obj_set_style_radius(card.bar, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(card.bar, 0, LV_PART_INDICATOR);
  lv_obj_set_style_border_width(card.bar, 0, LV_PART_MAIN);
  lv_obj_set_style_border_width(card.bar, 0, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(card.bar, lv_color_hex(0x111111), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(card.bar, (lv_opa_t)(255 * 0.42), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(card.bar, (lv_opa_t)(255 * 0.72), LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(card.bar, lv_color_hex(0x404040), LV_PART_INDICATOR);
  lv_bar_set_range(card.bar, 0, 100);
  lv_bar_set_value(card.bar, 50, LV_ANIM_OFF);

  lv_obj_t *speed_value = lv_obj_create(card.root);
  lv_obj_remove_style_all(speed_value);
  lv_obj_set_size(speed_value, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_style_pad_all(speed_value, 0, 0);
  lv_obj_set_style_bg_opa(speed_value, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_opa(speed_value, LV_OPA_TRANSP, 0);
  lv_obj_set_flex_flow(speed_value, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(speed_value, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);
  lv_obj_add_flag(speed_value, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
  lv_obj_add_flag(speed_value, LV_OBJ_FLAG_IGNORE_LAYOUT);
  lv_obj_align(speed_value, LV_ALIGN_CENTER, -6, -6);

  card.value_label = add_simple_label(speed_value, "50", speed_font,
                                      lv_color_white(), LV_OPA_COVER);
  card.unit_label = add_simple_label(speed_value, "km/h", small_font,
                                     lv_color_white(), LV_OPA_COVER);
  lv_obj_set_style_translate_y(card.unit_label,
                               get_baseline_offset(speed_font, small_font), 0);

  card.caption_label = add_simple_label(card.root, "MAX 100 km/h", small_font,
                                        lv_color_white(), (lv_opa_t)(255 * 0.55));
  lv_obj_add_flag(card.caption_label, LV_OBJ_FLAG_IGNORE_LAYOUT);
  lv_obj_align(card.caption_label, LV_ALIGN_BOTTOM_MID, 0, -4);

  add_title(card.root, "SPEED");
  return card;
}

static void speed_card_set_val(speed_card_t *card, int value, int max) {
  if (card == NULL || card->root == NULL || max <= 0) {
    return;
  }

  const int clamped = clamp_i32(value, 0, max);

  char txt[16];
  snprintf(txt, sizeof(txt), "%d", clamped);
  lv_label_set_text(card->value_label, txt);
  lv_bar_set_value(card->bar, clamped * 100 / max, LV_ANIM_OFF);
}

static height_card_t create_height_card(lv_obj_t *parent, int32_t w, int32_t h) {
  height_card_t card = {0};
  card.card_w = w;
  card.card_h = h;
  card.root = create_panel(parent, w, h, lv_color_black());
  if (card.root == NULL) {
    return card;
  }

  card.top_label = add_simple_label(card.root, "50 cm", small_font,
                                    lv_color_white(), (lv_opa_t)(255 * 0.56));
  lv_obj_add_flag(card.top_label, LV_OBJ_FLAG_IGNORE_LAYOUT);
  lv_obj_align(card.top_label, LV_ALIGN_TOP_LEFT, 14, 12);

  card.bottom_label = add_simple_label(card.root, "0 cm", small_font,
                                     lv_color_white(), (lv_opa_t)(255 * 0.56));
  lv_obj_add_flag(card.bottom_label, LV_OBJ_FLAG_IGNORE_LAYOUT);
  lv_obj_align(card.bottom_label, LV_ALIGN_BOTTOM_LEFT, 14, -12);

  card.track = lv_obj_create(card.root);
  lv_obj_remove_style_all(card.track);
  lv_obj_add_flag(card.track, LV_OBJ_FLAG_IGNORE_LAYOUT);
  lv_obj_set_pos(card.track, HEIGHT_TRACK_X, HEIGHT_TRACK_Y);
  lv_obj_set_size(card.track, HEIGHT_TRACK_W, HEIGHT_TRACK_H);
  lv_obj_set_style_bg_color(card.track, lv_color_white(), 0);
  lv_obj_set_style_bg_opa(card.track, (lv_opa_t)(255 * 0.25), 0);
  lv_obj_set_style_radius(card.track, 0, 0);

  card.target_marker = lv_obj_create(card.root);
  lv_obj_remove_style_all(card.target_marker);
  lv_obj_add_flag(card.target_marker, LV_OBJ_FLAG_IGNORE_LAYOUT);
  lv_obj_set_size(card.target_marker, 78, 8);
  lv_obj_set_style_bg_color(card.target_marker, lv_color_hex(0xFFB000), 0);
  lv_obj_set_style_bg_opa(card.target_marker, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(card.target_marker, 0, 0);

  card.current_marker = lv_obj_create(card.root);
  lv_obj_remove_style_all(card.current_marker);
  lv_obj_add_flag(card.current_marker, LV_OBJ_FLAG_IGNORE_LAYOUT);
  lv_obj_set_size(card.current_marker, 92, 8);
  lv_obj_set_style_bg_color(card.current_marker, lv_color_white(), 0);
  lv_obj_set_style_bg_opa(card.current_marker, (lv_opa_t)(255 * 0.92), 0);
  lv_obj_set_style_radius(card.current_marker, 0, 0);

  lv_obj_t *value_box = lv_obj_create(card.root);
  lv_obj_remove_style_all(value_box);
  lv_obj_set_size(value_box, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_style_pad_all(value_box, 0, 0);
  lv_obj_set_style_bg_opa(value_box, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_opa(value_box, LV_OPA_TRANSP, 0);
  lv_obj_set_flex_flow(value_box, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(value_box, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);
  lv_obj_add_flag(value_box, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
  lv_obj_add_flag(value_box, LV_OBJ_FLAG_IGNORE_LAYOUT);
  lv_obj_align(value_box, LV_ALIGN_RIGHT_MID, -18, -4);

  card.current_label = add_simple_label(value_box, "25", height_value_font,
                                        lv_color_white(), LV_OPA_COVER);
  lv_obj_t *unit = add_simple_label(value_box, "cm", small_font,
                                    lv_color_white(), LV_OPA_COVER);
  lv_obj_set_style_translate_y(unit,
                               get_baseline_offset(height_value_font, small_font), 0);

  card.target_label = add_simple_label(card.root, "target 30", small_font,
                                     lv_color_hex(0xFFB000), LV_OPA_COVER);
  lv_obj_add_flag(card.target_label, LV_OBJ_FLAG_IGNORE_LAYOUT);
  lv_obj_align(card.target_label, LV_ALIGN_RIGHT_MID, -18, 44);

  add_title(card.root, "HEIGHT");
  return card;
}

static void height_card_set_val(height_card_t *card, int current_cm,
                                int target_cm, int max_cm) {
  if (card == NULL || card->root == NULL || max_cm <= 0) {
    return;
  }

  current_cm = clamp_i32(current_cm, HEIGHT_MIN_CM, max_cm);
  target_cm = clamp_i32(target_cm, HEIGHT_MIN_CM, max_cm);

  char txt[16];
  snprintf(txt, sizeof(txt), "%d", current_cm);
  lv_label_set_text(card->current_label, txt);

  snprintf(txt, sizeof(txt), "target %d", target_cm);
  lv_label_set_text(card->target_label, txt);

  const int32_t current_y =
      HEIGHT_TRACK_Y + HEIGHT_TRACK_H - ((current_cm * HEIGHT_TRACK_H) / max_cm) - 4;
  const int32_t target_y =
      HEIGHT_TRACK_Y + HEIGHT_TRACK_H - ((target_cm * HEIGHT_TRACK_H) / max_cm) - 4;

  lv_obj_set_pos(card->current_marker, HEIGHT_TRACK_X - 24, current_y);
  lv_obj_set_pos(card->target_marker, HEIGHT_TRACK_X - 14, target_y);
}

static attitude_card_t create_attitude_card(lv_obj_t *parent, int32_t w, int32_t h) {
  attitude_card_t card = {0};
  card.card_w = w;
  card.card_h = h;
  card.root = create_panel(parent, w, h, lv_color_black());
  if (card.root == NULL) {
    return card;
  }

  lv_obj_set_style_clip_corner(card.root, true, 0);

  card.canvas = lv_canvas_create(card.root);
  if (card.canvas != NULL) {
    lv_obj_remove_style_all(card.canvas);
    lv_obj_add_flag(card.canvas, LV_OBJ_FLAG_IGNORE_LAYOUT);
    lv_obj_set_pos(card.canvas, 1, 1);
    lv_obj_set_size(card.canvas, w - 4, h - 4);
    const uint32_t stride = lv_draw_buf_width_to_stride(w - 4,
                                                        LV_COLOR_FORMAT_RGB565);
    const uint32_t buf_size = stride * (h - 4);
    card.draw_buf_mem = heap_caps_malloc(buf_size,
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (card.draw_buf_mem == NULL) {
      card.draw_buf_mem = heap_caps_malloc(buf_size, MALLOC_CAP_8BIT);
    }
    if (card.draw_buf_mem != NULL) {
      lv_canvas_set_buffer(card.canvas, card.draw_buf_mem, w - 4,
                           h - 4, LV_COLOR_FORMAT_RGB565);
    } else {
      ESP_LOGW(TAG, "Attitude canvas fell back to text-only mode");
    }
  }

  attitude_canvas_fill_background(&card);

  for (uint32_t i = 0; i < ATTITUDE_TICK_COUNT; i++) {
    char txt[8];
    snprintf(txt, sizeof(txt), "%d", (int)s_attitude_ticks[i]);
    card.pitch_marks[i] = create_pitch_label(card.root, txt);
  }

  card.heading_label = add_simple_label(card.root, "YAW 000", control_font,
                                    lv_color_white(), (lv_opa_t)(255 * 0.88));
  lv_obj_add_flag(card.heading_label, LV_OBJ_FLAG_IGNORE_LAYOUT);
  lv_obj_align(card.heading_label, LV_ALIGN_TOP_MID, 0, 6);

  card.roll_label = add_simple_label(card.root, "ROLL +0.0 deg", control_font,
                                     lv_color_white(), (lv_opa_t)(255 * 0.90));
  lv_obj_add_flag(card.roll_label, LV_OBJ_FLAG_IGNORE_LAYOUT);
  lv_obj_align(card.roll_label, LV_ALIGN_BOTTOM_LEFT, 8, -6);

  card.pitch_label = add_simple_label(card.root, "PITCH +0.0 deg", control_font,
                                      lv_color_white(), (lv_opa_t)(255 * 0.90));
  lv_obj_add_flag(card.pitch_label, LV_OBJ_FLAG_IGNORE_LAYOUT);
  lv_obj_align(card.pitch_label, LV_ALIGN_BOTTOM_RIGHT, -8, -6);

  add_title(card.root, "ATTITUDE");
  return card;
}

static void attitude_card_set_val(attitude_card_t *card, int roll_deg,
                                      int pitch_deg, int heading_deg) {
  if (card == NULL || card->root == NULL) {
    return;
  }

  const bool roll_clamped = (roll_deg < -45 || roll_deg > 45);
  const bool pitch_clamped = (pitch_deg < -25 || pitch_deg > 25);

  roll_deg = clamp_i32(roll_deg, -45, 45);
  pitch_deg = clamp_i32(pitch_deg, -25, 25);

  const int32_t heading = normalize_heading(heading_deg);
  card->roll_deg = roll_deg;
  card->pitch_deg = pitch_deg;
  card->heading_deg = heading;

  if (card->canvas != NULL && card->draw_buf_mem != NULL) {
    attitude_canvas_fill_background(card);
    lv_layer_t layer;
    lv_canvas_init_layer(card->canvas, &layer);
    attitude_canvas_draw(card, &layer);
    lv_canvas_finish_layer(card->canvas, &layer);
  }

  const int32_t canvas_w = card->card_w - 2;
  const int32_t canvas_h = card->card_h - 2;
  for (uint32_t i = 0; i < ATTITUDE_TICK_COUNT; i++) {
    lv_obj_t *mark = card->pitch_marks[i];
    if (mark == NULL) {
      continue;
    }

    const int32_t tick_pitch = s_attitude_ticks[i];
    const int32_t tick_y = -(tick_pitch * ATTITUDE_SCALE_PX);
    int32_t label_x;
    int32_t label_y;
    attitude_project_point(canvas_w / 2, canvas_h / 2, 0,
                           tick_y, pitch_deg * ATTITUDE_SCALE_PX, roll_deg,
                           &label_x, &label_y);

    const int32_t mark_w = lv_obj_get_width(mark);
    const int32_t mark_h = lv_obj_get_height(mark);
    lv_obj_set_pos(mark, label_x - (mark_w / 2), label_y - (mark_h / 2));
  }

  char txt[32];
  snprintf(txt, sizeof(txt), "YAW %03d", (int)heading);
  lv_label_set_text(card->heading_label, txt);

  snprintf(txt, sizeof(txt), "ROLL %+d", roll_deg);
  lv_label_set_text(card->roll_label, txt);
  lv_obj_set_style_text_color(card->roll_label,
                              roll_clamped ? lv_color_hex(0xFF0000) : lv_color_white(), 0);

  snprintf(txt, sizeof(txt), "PITCH %+d", pitch_deg);
  lv_label_set_text(card->pitch_label, txt);
  lv_obj_set_style_text_color(card->pitch_label,
                              pitch_clamped ? lv_color_hex(0xFF0000) : lv_color_white(), 0);
}

static battery_card_t create_battery_card(lv_obj_t *parent, int32_t w, int32_t h) {
  battery_card_t card = {0};
  card.card_w = w;
  card.card_h = h;
  card.root = create_panel(parent, w, h, lv_color_black());
  if (card.root == NULL) {
    return card;
  }

  card.bar = create_bar(card.root, w - 2, h - 2,
                        LV_BAR_MODE_NORMAL, LV_BAR_ORIENTATION_VERTICAL,
                        0, 100, lv_color_black(),
                        (lv_opa_t)(255 * 0.20));
  if (card.bar != NULL) {
    lv_obj_add_flag(card.bar, LV_OBJ_FLAG_IGNORE_LAYOUT);
    lv_obj_set_pos(card.bar, 0, 0);
    lv_obj_set_size(card.bar, lv_pct(100), lv_pct(100));
    lv_bar_set_value(card.bar, 50, LV_ANIM_OFF);
  }

  lv_obj_t *metrics_box = create_metrics_box(card.root);
  card.percent_label = add_simple_label(metrics_box, "50%", percent_font,
                                      lv_color_hex(0xFFD23F), LV_OPA_COVER);
  card.voltage_label = add_simple_label(metrics_box, "36V", control_font,
                                      lv_color_white(), LV_OPA_COVER);
  card.current_label = add_simple_label(metrics_box, "50A", control_font,
                                      lv_color_white(), LV_OPA_COVER);
  card.temp_label = add_simple_label(metrics_box, "50C", control_font,
                                    lv_color_hex(0xFF5A52), LV_OPA_COVER);

  add_title(card.root, "BATTERY");
  return card;
}

static void battery_card_set_val(battery_card_t *card, int percent,
                                   int voltage, int current, int temp) {
  if (card == NULL || card->root == NULL) {
    return;
  }

  const int32_t clamped = clamp_i32(percent, 0, 100);
  if (card->bar != NULL) {
    lv_bar_set_value(card->bar, (int)clamped, LV_ANIM_OFF);
  }

  char txt[32];
  snprintf(txt, sizeof(txt), "%d%%", (int)clamped);
  lv_label_set_text(card->percent_label, txt);
  apply_percent_color(card->bar, card->percent_label, clamped);

  snprintf(txt, sizeof(txt), "%dV", voltage);
  lv_label_set_text(card->voltage_label, txt);

  snprintf(txt, sizeof(txt), "%dA", current);
  lv_label_set_text(card->current_label, txt);

  snprintf(txt, sizeof(txt), "%dC", temp);
  lv_label_set_text(card->temp_label, txt);
}

static motor_card_t create_motor_card(lv_obj_t *parent, int32_t w, int32_t h) {
  motor_card_t card = {0};
  card.card_w = w;
  card.card_h = h;
  card.root = create_panel(parent, w, h, lv_color_black());
  if (card.root == NULL) {
    return card;
  }

  card.bar = create_bar(card.root, 1, 1, LV_BAR_MODE_NORMAL,
                        LV_BAR_ORIENTATION_VERTICAL, 0, 100,
                        lv_color_black(),
                        (lv_opa_t)(255 * 0.20));
  if (card.bar != NULL) {
    lv_obj_add_flag(card.bar, LV_OBJ_FLAG_IGNORE_LAYOUT);
    lv_obj_set_pos(card.bar, 0, 0);
    lv_obj_set_size(card.bar, lv_pct(100), lv_pct(100));
    lv_bar_set_value(card.bar, 50, LV_ANIM_OFF);
  }

  lv_obj_t *metrics_box = create_metrics_box(card.root);
  card.percent_label = add_simple_label(metrics_box, "50%", percent_font,
                                      lv_color_white(), LV_OPA_COVER);
  card.power_label = add_simple_label(metrics_box, "10.0 kW", control_font,
                                    lv_color_white(), LV_OPA_COVER);
  card.rpm_label = add_simple_label(metrics_box, "5000 RPM", control_font,
                                  lv_color_white(), LV_OPA_COVER);
  card.temp_label = add_simple_label(metrics_box, "50C", control_font,
                                    lv_color_hex(0x1390FF), LV_OPA_COVER);

  add_title(card.root, "MOTOR");
  return card;
}

static void motor_card_set_val(motor_card_t *card, int percent,
                                 int power_kw_x10, int rpm, int temp) {
  if (card == NULL || card->root == NULL) {
    return;
  }

  const int32_t clamped = clamp_i32(percent, 0, 100);
  if (card->bar != NULL) {
    lv_bar_set_value(card->bar, (int)clamped, LV_ANIM_OFF);
  }

  char txt[32];
  snprintf(txt, sizeof(txt), "%d%%", (int)clamped);
  lv_label_set_text(card->percent_label, txt);
  apply_percent_color(card->bar, card->percent_label, clamped);

  snprintf(txt, sizeof(txt), "%d.%d kW", power_kw_x10 / 10,
           (power_kw_x10 < 0 ? -power_kw_x10 : power_kw_x10) % 10);
  lv_label_set_text(card->power_label, txt);

  snprintf(txt, sizeof(txt), "%d RPM", rpm);
  lv_label_set_text(card->rpm_label, txt);

  snprintf(txt, sizeof(txt), "%dC", temp);
  lv_label_set_text(card->temp_label, txt);
}

static control_surface_card_t create_control_surface_card(
    lv_obj_t *parent, int32_t w, int32_t h, const char *title,
    bool vertical) {
  control_surface_card_t card = {0};
  card.card_w = w;
  card.card_h = h;
  card.root = create_panel(parent, w, h, lv_color_black());
  if (card.root == NULL) {
    return card;
  }

  add_title(card.root, title);

  if (vertical) {
    card.bar = create_bar(card.root, 18, 72, LV_BAR_MODE_SYMMETRICAL,
                          LV_BAR_ORIENTATION_VERTICAL, -15, 15,
                          lv_color_white(), (lv_opa_t)(255 * 0.92));
    if (card.bar != NULL) {
      lv_bar_set_value(card.bar, 0, LV_ANIM_OFF);
      lv_obj_add_flag(card.bar, LV_OBJ_FLAG_IGNORE_LAYOUT);
      lv_obj_set_pos(card.bar, (w - 18) / 2, 24);
    }

    card.zero_mark = lv_obj_create(card.root);
    lv_obj_remove_style_all(card.zero_mark);
    lv_obj_add_flag(card.zero_mark, LV_OBJ_FLAG_IGNORE_LAYOUT);
    lv_obj_set_pos(card.zero_mark, (w - 28) / 2, 60);
    lv_obj_set_size(card.zero_mark, 28, 6);
    lv_obj_set_style_bg_color(card.zero_mark, lv_color_hex(0x808080), 0);
    lv_obj_set_style_bg_opa(card.zero_mark, (lv_opa_t)(255 * 0.65), 0);
  } else {
    card.bar = create_bar(card.root, 84, 20, LV_BAR_MODE_SYMMETRICAL,
                          LV_BAR_ORIENTATION_HORIZONTAL, -20, 20,
                          lv_color_white(), (lv_opa_t)(255 * 0.95));
    if (card.bar != NULL) {
      lv_bar_set_value(card.bar, 0, LV_ANIM_OFF);
      lv_obj_add_flag(card.bar, LV_OBJ_FLAG_IGNORE_LAYOUT);
      lv_obj_set_pos(card.bar, (w - 84) / 2, 56);
    }

    card.zero_mark = lv_obj_create(card.root);
    lv_obj_remove_style_all(card.zero_mark);
    lv_obj_add_flag(card.zero_mark, LV_OBJ_FLAG_IGNORE_LAYOUT);
    lv_obj_set_pos(card.zero_mark, (w - 6) / 2, 56 - 4);
    lv_obj_set_size(card.zero_mark, 6, 28);
    lv_obj_set_style_bg_color(card.zero_mark, lv_color_hex(0x808080), 0);
    lv_obj_set_style_bg_opa(card.zero_mark, (lv_opa_t)(255 * 0.65), 0);
  }

  card.value_label = add_simple_label(card.root, "0 deg", control_font,
                                      lv_color_white(), LV_OPA_COVER);
  lv_obj_add_flag(card.value_label, LV_OBJ_FLAG_IGNORE_LAYOUT);
  lv_obj_align(card.value_label, LV_ALIGN_BOTTOM_MID, 0, -8);
  return card;
}

static void control_surface_card_set_val(control_surface_card_t *card,
                                         int angle_deg) {
  if (card == NULL || card->root == NULL) {
    return;
  }

  char txt[16];
  snprintf(txt, sizeof(txt), "%d deg", angle_deg);
  lv_label_set_text(card->value_label, txt);
  if (card->bar != NULL) {
    lv_bar_set_value(card->bar, angle_deg, LV_ANIM_OFF);
  }
}

dashboard_ui_t *dashboard_ui_init(lv_obj_t *screen, font_get_cb_t font_get_cb,
                                  void *font_get_user_data) {
  if (screen == NULL) {
    return NULL;
  }

  lv_display_t *disp = lv_display_get_default();
  int32_t h_res = lv_obj_get_width(screen);
  int32_t v_res = lv_obj_get_height(screen);
  if (h_res <= 0 || v_res <= 0) {
    h_res = disp ? lv_display_get_horizontal_resolution(disp) : 1024;
    v_res = disp ? lv_display_get_vertical_resolution(disp) : 600;
  }

  ESP_LOGI(TAG, "Display resolution: %dx%d", (int)h_res, (int)v_res);

  init_dashboard_fonts(font_get_cb, font_get_user_data);
  configure_screen(screen);

  dashboard_ui_t *ui = calloc(1, sizeof(*ui));
  if (ui == NULL) {
    ESP_LOGE(TAG, "Failed to allocate dashboard UI state");
    return NULL;
  }

  /* Reset primitive throttles for this UI instance. */
  for (size_t i = 0; i < (size_t)kUiThrottleCount; i++) {
    ui->last_update_us[i] = 0;
  }

  int32_t ui_h_res = h_res;
  int32_t ui_v_res = v_res;

  ui->h_res = ui_h_res;
  ui->v_res = ui_v_res;

  ui->root = create_dashboard_root(screen, ui_h_res, ui_v_res);
  if (ui->root == NULL) {
    ESP_LOGE(TAG, "Failed to create dashboard root container");
    free(ui);
    return NULL;
  }

  lv_obj_set_pos(ui->root, 0, 0);

  /* Compute exact card widths so each row fills the display width.
     With pad_column=-1, adjacent cards overlap by 1 px, so the total
     physical width needed is h_res + (n - 1) for n cards.
     Row 2 widths are derived from row 1 so vertical borders align. */
  int32_t row1_w[3];
  int32_t row1_total = ui_h_res + 2;
  int32_t row1_base = row1_total / 3;
  int32_t row1_rem = row1_total % 3;
  for (int i = 0; i < 3; i++) {
    row1_w[i] = row1_base + (i < row1_rem ? 1 : 0);
  }

  int32_t row2_w[6];
  for (int i = 0; i < 3; i++) {
    int32_t pair_total = row1_w[i] + 1;
    row2_w[2 * i] = pair_total / 2;
    row2_w[2 * i + 1] = pair_total - row2_w[2 * i];
  }
  int32_t row2_h = row1_base / 2;

  ui->speed = create_speed_card(ui->root, row1_w[0], row1_w[0]);
  ui->height = create_height_card(ui->root, row1_w[1], row1_w[1]);
  ui->attitude = create_attitude_card(ui->root, row1_w[2], row1_w[2]);
  ui->battery = create_battery_card(ui->root, row2_w[0], row2_h);
  ui->motors[0] = create_motor_card(ui->root, row2_w[1], row2_h);
  ui->motors[1] = create_motor_card(ui->root, row2_w[2], row2_h);
  ui->rudder = create_control_surface_card(ui->root, row2_w[3], row2_h,
                                            "RUDDER", false);
  ui->elevon_left = create_control_surface_card(ui->root, row2_w[4], row2_h,
                                                  "L ELEVON", true);
  ui->elevon_right = create_control_surface_card(ui->root, row2_w[5], row2_h,
                                                   "R ELEVON", true);

  if (ui->speed.root == NULL || ui->height.root == NULL || ui->attitude.root == NULL ||
      ui->battery.root == NULL || ui->motors[0].root == NULL || ui->motors[1].root == NULL ||
      ui->rudder.root == NULL || ui->elevon_left.root == NULL || ui->elevon_right.root == NULL) {
    ESP_LOGE(TAG, "Failed to create one or more dashboard cards");
    dashboard_ui_destroy(ui);
    return NULL;
  }

  return ui;
}

void dashboard_ui_destroy(dashboard_ui_t *ui) {
  if (ui == NULL) {
    return;
  }

  if (ui->root != NULL) {
    lv_obj_delete(ui->root);
    ui->root = NULL;
  }

  if (ui->attitude.draw_buf_mem != NULL) {
    heap_caps_free(ui->attitude.draw_buf_mem);
    ui->attitude.draw_buf_mem = NULL;
  }

  free(ui);
}

lv_obj_t *dashboard_ui_create_status_strip(lv_obj_t *screen, int32_t h_res, int32_t strip_h_px) {
  if (screen == NULL || h_res <= 0 || strip_h_px <= 0) {
    return NULL;
  }

  lv_obj_t *strip_bg = lv_obj_create(screen);
  lv_obj_remove_style_all(strip_bg);
  lv_obj_remove_flag(strip_bg, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(strip_bg, h_res, strip_h_px);
  lv_obj_align(strip_bg, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_set_style_bg_color(strip_bg, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(strip_bg, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(strip_bg, 0, 0);

  lv_obj_t *label = lv_label_create(strip_bg);
  lv_obj_set_size(label, h_res - 6, strip_h_px - 2);
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(label, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(label, LV_OPA_COVER, 0);
  lv_obj_set_style_text_color(label, lv_color_hex(0xCFCFCF), 0);
  lv_label_set_long_mode(label, LV_LABEL_LONG_MODE_SCROLL_CIRCULAR);
  lv_label_set_text(label, "…");
  return label;
}

void dashboard_ui_set_speed(dashboard_ui_t *ui, int32_t speed_kmh) {
  if (ui == NULL) {
    return;
  }
  if (!ui_throttle_ok(ui, kUiSpeed)) {
    return;
  }
  speed_card_set_val(&ui->speed, speed_kmh, 100);
}

void dashboard_ui_set_height(dashboard_ui_t *ui, int32_t height_cm, int32_t height_target_cm) {
  if (ui == NULL) {
    return;
  }
  if (!ui_throttle_ok(ui, kUiHeight)) {
    return;
  }
  height_card_set_val(&ui->height, height_cm, height_target_cm, 50);
}

void dashboard_ui_set_attitude(dashboard_ui_t *ui, int32_t roll_deg, int32_t pitch_deg, int32_t heading_deg) {
  if (ui == NULL) {
    return;
  }
  if (!ui_throttle_ok(ui, kUiAttitude)) {
    return;
  }
  attitude_card_set_val(&ui->attitude, roll_deg, pitch_deg, heading_deg);
}

void dashboard_ui_set_battery(dashboard_ui_t *ui, int32_t percent, int32_t voltage_v, int32_t current_a, int32_t temp_c) {
  if (ui == NULL) {
    return;
  }
  if (!ui_throttle_ok(ui, kUiBattery)) {
    return;
  }
  battery_card_set_val(&ui->battery, percent, voltage_v, current_a, temp_c);
}

void dashboard_ui_set_motor(dashboard_ui_t *ui, int32_t index, int32_t percent, int32_t power_kw_x10, int32_t rpm, int32_t temp_c) {
  if (ui == NULL || index < 0 || index >= DASHBOARD_MOTOR_MAX) {
    return;
  }
  /* Motor updates tend to move together; throttle them as one logical section. */
  if (index == 0 && !ui_throttle_ok(ui, kUiMotor0)) {
    return;
  }
  if (index == 1 && !ui_throttle_ok(ui, kUiMotor1)) {
    return;
  }
  motor_card_set_val(&ui->motors[index], percent, power_kw_x10, rpm, temp_c);
}

void dashboard_ui_set_rudder(dashboard_ui_t *ui, int32_t rudder_deg) {
  if (ui == NULL) {
    return;
  }
  if (!ui_throttle_ok(ui, kUiRudder)) {
    return;
  }
  control_surface_card_set_val(&ui->rudder, rudder_deg);
}

void dashboard_ui_set_elevons(dashboard_ui_t *ui, int32_t left_deg, int32_t right_deg) {
  if (ui == NULL) {
    return;
  }
  if (!ui_throttle_ok(ui, kUiElevons)) {
    return;
  }
  control_surface_card_set_val(&ui->elevon_left, left_deg);
  control_surface_card_set_val(&ui->elevon_right, right_deg);
}

void dashboard_ui_apply_data(dashboard_ui_t *ui, const dashboard_data_t *data) {
  if (ui == NULL || data == NULL) {
    return;
  }

  dashboard_ui_set_speed(ui, data->speed_kmh);
  dashboard_ui_set_height(ui, data->height_cm, data->height_target_cm);
  dashboard_ui_set_attitude(ui, data->roll_deg, data->pitch_deg, data->heading_deg);
  dashboard_ui_set_battery(ui, data->battery_percent, data->battery_voltage_v,
                            data->battery_current_a, data->battery_temp_c);

  const int32_t motor_count = (data->motor_count < 0) ? 0 : data->motor_count;
  const int32_t max_motors = (motor_count > DASHBOARD_MOTOR_MAX) ? DASHBOARD_MOTOR_MAX : motor_count;
  for (int32_t i = 0; i < max_motors; i++) {
    dashboard_ui_set_motor(ui, i, data->motor_percent[i], data->motor_power_kw_x10[i],
                              data->motor_rpm[i], data->motor_temp_c[i]);
  }

  dashboard_ui_set_rudder(ui, data->rudder_deg);
  dashboard_ui_set_elevons(ui, data->elevon_left_deg, data->elevon_right_deg);
}
