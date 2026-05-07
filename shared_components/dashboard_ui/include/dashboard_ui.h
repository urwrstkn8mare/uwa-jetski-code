#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "lvgl.h"

#define DASHBOARD_MOTOR_MAX 2

/* Complete dashboard state shared by the demo and CAN update paths.
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

typedef const lv_font_t *(*font_get_cb_t)(uint16_t size_px, int weight, void *user_data);

/* Display lock callbacks: acquire and release the display lock before/after LVGL operations.
 * This ensures thread-safe access to LVGL objects. */
typedef void (*dashboard_ui_lock_fn_t)(void);
typedef void (*dashboard_ui_unlock_fn_t)(void);

/** Configuration for dashboard_ui initialization.
 * Lock callbacks are required to ensure thread-safe LVGL access. */
typedef struct {
  lv_obj_t *screen;                      /**< Parent LVGL screen object (required) */
  font_get_cb_t font_get_cb;             /**< Font loading callback (optional) */
  void *font_get_user_data;              /**< User data for font callback (optional) */
  dashboard_ui_lock_fn_t lock_cb;        /**< Display lock acquire function (required) */
  dashboard_ui_unlock_fn_t unlock_cb;    /**< Display lock release function (required) */
} dashboard_ui_cfg_t;

/* Initialize the singleton dashboard_ui instance.
 * Can only be called once; subsequent calls return ESP_ERR_INVALID_STATE.
 * Must be paired with dashboard_ui_destroy(). */
esp_err_t dashboard_ui_init(const dashboard_ui_cfg_t *cfg);

/* Clean up the singleton dashboard_ui instance. */
void dashboard_ui_destroy(void);

/* Apply a full dashboard snapshot through the per-box setters.
 * Each setter automatically acquires the display lock. */
void dashboard_ui_apply_data(const dashboard_data_t *data);

/* Per-box partial updates — use these when only a subset of data changes
 * to avoid redundant LVGL redraws of untouched cards.
 * Each function automatically acquires the display lock internally. */
void dashboard_ui_set_speed(int32_t speed_kmh);
void dashboard_ui_set_height(int32_t height_cm, int32_t height_target_cm);
void dashboard_ui_set_attitude(int32_t roll_deg, int32_t pitch_deg, int32_t heading_deg);
void dashboard_ui_set_battery(int32_t percent, int32_t voltage_v, int32_t current_a, int32_t temp_c);
void dashboard_ui_set_motor(int32_t index, int32_t percent, int32_t power_kw_x10, int32_t rpm, int32_t temp_c);
void dashboard_ui_set_rudder(int32_t rudder_deg);
void dashboard_ui_set_elevons(int32_t left_deg, int32_t right_deg);
