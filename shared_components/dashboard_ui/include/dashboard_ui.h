#pragma once

#include <stdint.h>

#include "lvgl.h"

#define DASHBOARD_MOTOR_MAX 2

/* Complete dashboard state. Fill this struct and pass to dashboard_ui_set_data().
 * All angles are in degrees; all temperatures in Celsius. */
typedef struct {
  int32_t speed_kmh;              /**< 0..100 */
  int32_t height_cm;              /**< 0..50  current ride height */
  int32_t height_target_cm;       /**< 0..50  target ride height */
  int32_t roll_deg;               /**< -45..45 */
  int32_t pitch_deg;              /**< -25..25 */
  int32_t heading_deg;            /**< 0..359 compass heading */
  int32_t battery_percent;        /**< 0..100 */
  int32_t battery_voltage_v;      /**< displayed as "XXV" */
  int32_t battery_current_a;      /**< displayed as "XXA" */
  int32_t battery_temp_c;         /**< displayed as "XXC" */
  int32_t motor_count;            /**< 0..DASHBOARD_MOTOR_MAX  motors shown */
  int32_t motor_percent[DASHBOARD_MOTOR_MAX];       /**< 0..100 throttle */
  int32_t motor_power_kw_x10[DASHBOARD_MOTOR_MAX];  /**< displayed "XX.X kW" */
  int32_t motor_rpm[DASHBOARD_MOTOR_MAX];           /**< displayed "XXXX RPM" */
  int32_t motor_temp_c[DASHBOARD_MOTOR_MAX];        /**< displayed "XXC" */
  int32_t rudder_deg;            /**< -20..20 */
  int32_t elevon_left_deg;       /**< -15..15 */
  int32_t elevon_right_deg;      /**< -15..15 */
} dashboard_data_t;

typedef struct dashboard_ui dashboard_ui_t;

typedef const lv_font_t *(*font_get_cb_t)(uint16_t size_px, int weight, void *user_data);

void dashboard_ui_set_font_callback(font_get_cb_t cb, void *user_data);

dashboard_ui_t *dashboard_ui_create(lv_obj_t *screen);
void dashboard_ui_destroy(dashboard_ui_t *ui);
void dashboard_ui_set_data(dashboard_ui_t *ui, const dashboard_data_t *data);

/* Per-box partial updates — use these when only a subset of data changes
 * to avoid redundant LVGL redraws of untouched cards. */
void dashboard_ui_set_speed(dashboard_ui_t *ui, int32_t speed_kmh);
void dashboard_ui_set_height(dashboard_ui_t *ui, int32_t height_cm, int32_t height_target_cm);
void dashboard_ui_set_attitude(dashboard_ui_t *ui, int32_t roll_deg, int32_t pitch_deg, int32_t heading_deg);
void dashboard_ui_set_battery(dashboard_ui_t *ui, int32_t percent, int32_t voltage_v, int32_t current_a, int32_t temp_c);
void dashboard_ui_set_motor(dashboard_ui_t *ui, int32_t index, int32_t percent, int32_t power_kw_x10, int32_t rpm, int32_t temp_c);
void dashboard_ui_set_rudder(dashboard_ui_t *ui, int32_t rudder_deg);
void dashboard_ui_set_elevons(dashboard_ui_t *ui, int32_t left_deg, int32_t right_deg);
