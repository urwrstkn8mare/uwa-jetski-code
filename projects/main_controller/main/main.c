#include "can.h"
#include "can_ids.h"
#include "config.h"
#include "control.h"
#include "encoder_can.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "height.h"
#include "imu.h"
#include "lvgl.h"
#include "servo_drive.h"
#include "status_ui.h"
#include "t_display_s3.h"
#include "webui.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>

static const char *TAG = "main";

static servo_channel_t s_servo_left;
static servo_channel_t s_servo_right;
static const int SERVO_GPIO_LEFT  = 1;
static const int SERVO_GPIO_RIGHT = 2;

/* Latest joystick values from aux controller (0..100, 50 = centre) */
static volatile uint16_t s_joy_bank_pct  = 50;
static volatile uint16_t s_joy_pitch_pct = 50;

static void on_can_rx(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp) {
  (void)timestamp;
  encoder_can_on_rx(buffer, header_id);

  if (header_id == CAN_ID_JOYSTICK && buffer != NULL) {
    can_joystick_t joy;
    memcpy(&joy, buffer, sizeof(joy));
    s_joy_bank_pct  = (joy.bank_pct  > 100u) ? 100u : joy.bank_pct;
    s_joy_pitch_pct = (joy.pitch_pct > 100u) ? 100u : joy.pitch_pct;
  }
}

static void on_can_status(const char *line) {
  status_ui_update("CAN", "%s", line);
}

/* Map encoder angle to rudder percentage (0..100, 50 = centre). */
static uint16_t encoder_angle_to_pct(float angle_deg) {
  const float max_ang = (float)CONFIG_ENCODER_MAX_ANGLE_DEG;
  float norm = angle_deg / max_ang;
  if (norm >  1.0f) norm =  1.0f;
  if (norm < -1.0f) norm = -1.0f;
  return (uint16_t)((norm + 1.0f) * 50.0f);
}

/* Set both elevons directly (manual / unarmed mode). */
static void set_elevons_direct(float left_deg, float right_deg) {
  if (s_servo_left != SERVO_CHANNEL_INVALID) {
    servo_drive_set_degrees(s_servo_left, left_deg);
  }
  if (s_servo_right != SERVO_CHANNEL_INVALID) {
    servo_drive_set_degrees(s_servo_right, right_deg);
  }
}

typedef struct {
  uint32_t min_us;
  uint32_t max_us;
  uint64_t sum_us;
  uint32_t count;
} perf_stats_t;

static void perf_tick(perf_stats_t *s, uint32_t dur_us, bool is_armed) {
  if (dur_us < s->min_us) s->min_us = dur_us;
  if (dur_us > s->max_us) s->max_us = dur_us;
  s->sum_us += dur_us;
  s->count++;

  if (s->count < 50u) {
    return;
  }

  uint32_t avg_us = (uint32_t)(s->sum_us / s->count);
  uint32_t avg_hz = avg_us > 0 ? (1000000u / avg_us) : 0;

  status_ui_update(is_armed ? "perf_armed" : "perf_disarmed",
                   "iter %"PRIu32"/%"PRIu32"us avg %"PRIu32"Hz",
                   avg_us, s->max_us, avg_hz);

  can_ctrl_perf_t perf = {
      .iter_avg_us = (avg_us      > 0xFFFFu) ? 0xFFFFu : (uint16_t)avg_us,
      .iter_max_us = (s->max_us   > 0xFFFFu) ? 0xFFFFu : (uint16_t)s->max_us,
      .iter_hz     = (avg_hz      > 0xFFFFu) ? 0xFFFFu : (uint16_t)avg_hz,
      .is_armed    = is_armed ? 1u : 0u,
      ._pad        = 0,
  };
  (void)can_tx(CAN_ID_CTRL_PERF, (const uint8_t *)&perf, sizeof(perf));

  /* Reset accumulators; keep min/max across full session */
  s->sum_us = 0;
  s->count  = 0;
}

/* ── 50 Hz control task ── */
static void ctrl_task(void *arg) {
  (void)arg;

  perf_stats_t s_armed_perf    = {.min_us = UINT32_MAX};
  perf_stats_t s_disarmed_perf = {.min_us = UINT32_MAX};

  for (;;) {
    int64_t t0 = esp_timer_get_time();

    if (control_is_armed() && !servo_drive_any_cal_mode()) {
      /* ── Armed: full PID stabilisation loop ── */

      uint16_t rudder_pct = 50;
      float encoder_angle = 0.0f;
      if (encoder_can_is_fresh(500, &encoder_angle)) {
        rudder_pct = encoder_angle_to_pct(encoder_angle);
      }

      float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
      if (imu_is_ready()) {
        imu_get_pitch_roll_yaw(&pitch, &roll, &yaw);
      }

      int32_t height_cm = 30;
      height_get_cm(&height_cm);

      control_output_t out;
      control_update((int16_t)height_cm, pitch, roll, rudder_pct,
                     s_joy_pitch_pct, &out);

      if (s_servo_left != SERVO_CHANNEL_INVALID) {
        servo_drive_set_degrees(s_servo_left, out.elevon_left_deg);
      }
      if (s_servo_right != SERVO_CHANNEL_INVALID) {
        servo_drive_set_degrees(s_servo_right, out.elevon_right_deg);
      }

      perf_tick(&s_armed_perf, (uint32_t)(esp_timer_get_time() - t0), true);

    } else {
      /* ── Disarmed: joystick → elevons directly ── */

      if (servo_drive_any_cal_mode() && control_is_armed()) {
        control_disarm();
      }

      float bank_norm  = ((float)s_joy_bank_pct  / 50.0f) - 1.0f;
      float pitch_norm = ((float)s_joy_pitch_pct / 50.0f) - 1.0f;
      const float MAX_JOY_DEG = 15.0f;
      float pitch_cmd = pitch_norm * MAX_JOY_DEG;
      float diff_cmd  = bank_norm  * MAX_JOY_DEG;
      set_elevons_direct(pitch_cmd + diff_cmd, pitch_cmd - diff_cmd);

      perf_tick(&s_disarmed_perf, (uint32_t)(esp_timer_get_time() - t0), false);
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void app_main(void) {
  tdisplays3_handle_t disp_board;
  if (tdisplays3_init(&disp_board) == ESP_OK) {
    if (tdisplays3_display_lock(200)) {
      lv_obj_t *scr = lv_screen_active();
      lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), LV_PART_MAIN);
      lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
      tdisplays3_display_unlock();
    }
    const status_ui_cfg_t ui_cfg = {
        .parent          = lv_screen_active(),
        .lock_cb         = tdisplays3_display_lock,
        .unlock_cb       = tdisplays3_display_unlock,
        .lock_timeout_ms = 200,
        .min_interval_ms = 200,
    };
    if (status_ui_start(&ui_cfg) != ESP_OK) {
      ESP_LOGW(TAG, "status_ui_start failed — serial only");
    }
  } else {
    ESP_LOGW(TAG, "tdisplays3_init failed — serial only");
  }

  config_init();
  app_config_t cfg;
  config_load(&cfg);
  control_init(&cfg.control);

  if (can_init(on_can_rx) != ESP_OK) {
    ESP_LOGW(TAG, "CAN init failed — CAN TX disabled");
  }
  can_register_status_cb(on_can_status);

  encoder_can_init();

  (void)servo_drive_init_hw();
  s_servo_left  = servo_drive_open(SERVO_GPIO_LEFT);
  s_servo_right = servo_drive_open(SERVO_GPIO_RIGHT);
  if (s_servo_left != SERVO_CHANNEL_INVALID) {
    servo_drive_apply_cal(s_servo_left, &cfg.servo.channel[0]);
  }
  if (s_servo_right != SERVO_CHANNEL_INVALID) {
    servo_drive_apply_cal(s_servo_right, &cfg.servo.channel[1]);
  }
  if (s_servo_left == SERVO_CHANNEL_INVALID && s_servo_right == SERVO_CHANNEL_INVALID) {
    ESP_LOGW(TAG, "All servos in simulated mode");
  }

  if (xTaskCreate(ctrl_task, "ctrl", 8192, NULL, 2, NULL) != pdPASS) {
    ESP_LOGE(TAG, "ctrl task create failed");
  }

  if (imu_init() != ESP_OK) {
    ESP_LOGW(TAG, "IMU off — no attitude CAN");
  }

  (void)height_init();

  webui_start();
}
