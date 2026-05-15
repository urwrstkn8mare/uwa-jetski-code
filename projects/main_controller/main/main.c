#include "app_state.h"
#include "can.h"
#include "can_ids.h"
#include "config.h"
#include "control.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "height.h"
#include "imu.h"
#include "servo_drive.h"
#include "status_ui.h"
#include "t_display_s3.h"
#include "webui.h"

#include "lvgl.h"

#include <math.h>
#include <string.h>

static const auto TAG = "main";

static tdisplays3_handle_t s_tdisp_board;

static void main_status_lock(void) { tdisplays3_display_lock(200); }

static void main_status_unlock(void) { tdisplays3_display_unlock(); }

static void main_status_display_init(void) {
  app_state_set_display(false);
  if (tdisplays3_init(&s_tdisp_board) != ESP_OK) {
    ESP_LOGW(TAG, "tdisplays3_init failed — serial only");
    return;
  }
  if (s_tdisp_board.display == NULL) {
    ESP_LOGW(TAG, "Display handle null — serial only");
    return;
  }
  app_state_set_display(true);

  lv_obj_t *scr = NULL;
  if (tdisplays3_display_lock(200)) {
    scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
    tdisplays3_display_unlock();
  }

  const status_ui_cfg_t cfg = {
      .parent = scr,
      .lock_cb = main_status_lock,
      .unlock_cb = main_status_unlock,
      .min_interval_ms = 200,
  };
  if (status_ui_start(&cfg) != ESP_OK) {
    ESP_LOGW(TAG, "status display init failed");
  }
  ESP_LOGI(TAG, "Display debug label ready");
}

/* ── Servo handles (created at startup) ── */
static servo_channel_t s_servo_left;
static servo_channel_t s_servo_right;

static const int SERVO_GPIO_LEFT  = 1;
static const int SERVO_GPIO_RIGHT = 2;

/* ── CAN RX callback ── */
static void on_can_rx(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp) {
  (void)timestamp;
  app_state_on_can_rx(buffer, header_id);
}

/* ── 50 Hz orchestrator task ── */
static void ctrl_task(void *arg) {
  (void)arg;
  for (;;) {
    app_state_t st;
    app_state_get(&st);

    uint16_t pot_pct = 50;
    if (!app_state_pot_fresh(500, &pot_pct)) {
      pot_pct = 50;
    }

    /* Read sensors */
    float pitch = 0, roll = 0, yaw = 0;
    bool have_imu = false;
    if (st.imu_ok) {
      have_imu = (imu_get_pitch_roll_yaw(&pitch, &roll, &yaw) == ESP_OK);
    }

    int32_t height_cm = 30;
    bool have_height = false;
    if (height_get_cm(&height_cm) == ESP_OK) {
      have_height = true;
    }

    /* Run control loop */
    control_output_t ctrl_out;
    control_update((int16_t)height_cm, pitch, roll, pot_pct, &ctrl_out);

    /* Drive servos */
    if (servo_drive_any_cal_mode() && control_is_armed()) {
      control_disarm();
    }

    if (!servo_drive_any_cal_mode()) {
      if (s_servo_left != SERVO_CHANNEL_INVALID) {
        servo_drive_set_degrees(s_servo_left, ctrl_out.elevon_left_deg);
      }
      if (s_servo_right != SERVO_CHANNEL_INVALID) {
        servo_drive_set_degrees(s_servo_right, ctrl_out.elevon_right_deg);
      }
    }

    /* ── CAN TX ── */
    if (can_is_ready()) {
      if (have_imu) {
        can_attitude_t att = {
            .pitch_deg = (int16_t)lroundf(pitch),
            .roll_deg = (int16_t)lroundf(roll),
            .yaw_deg = (int16_t)lroundf(yaw),
        };
        (void)can_tx(CAN_ID_ATTITUDE, (const uint8_t *)&att, sizeof(att));
      }

      if (have_height) {
        if (height_cm >= 0 && height_cm <= (int32_t)UINT16_MAX) {
          can_height_t hb = {.height_cm = (uint16_t)height_cm};
          (void)can_tx(CAN_ID_HEIGHT, (const uint8_t *)&hb, sizeof(hb));
        }
      }

      float ldeg = 0.0f, rdeg = 0.0f;
      if (s_servo_left != SERVO_CHANNEL_INVALID) {
        servo_drive_get_commanded_degrees(s_servo_left, &ldeg);
      }
      if (s_servo_right != SERVO_CHANNEL_INVALID) {
        servo_drive_get_commanded_degrees(s_servo_right, &rdeg);
      }
      can_servo_pos_t sp = {
          .channel_a_deg = ldeg,
          .channel_b_deg = rdeg,
      };
      (void)can_tx(CAN_ID_SERVO_POS, (const uint8_t *)&sp, sizeof(sp));

      can_ctrl_status_t cs = {
          .height_target_cm = control_get_target(),
          .height_current_cm = (int16_t)height_cm,
          .pitch_deg = (int8_t)lroundf(pitch),
          .roll_deg = (int8_t)lroundf(roll),
          .flags = ctrl_out.armed ? 1u : 0u,
      };
      (void)can_tx(CAN_ID_CTRL_STATUS, (const uint8_t *)&cs, sizeof(cs));
    }

    /* ── status UI ── */
    if (can_is_ready()) {
      char buf[128];
      int n = can_snprintf_board_status(buf, sizeof(buf));
      if (n > 0) {
        status_ui_update("CAN", "%s", buf);
      }
    } else {
      status_ui_update("CAN", "off (TWAI down)");
    }

    {
      char buf[128];
      size_t n = app_state_debug_flags_line_write(buf, sizeof(buf));
      if (n > 0) {
        status_ui_update("Flags", "%s", buf);
      }
    }

    uint16_t pot = 50;
    bool pot_fresh = app_state_pot_fresh(500, &pot);
    status_ui_update("Rudder",
                     "%s %u%%",
                     pot_fresh ? "CAN" : "DEMO", (unsigned)pot);

    status_ui_update("Control",
                     "%s target=%d ht=%ld L=%.1f R=%.1f",
                     ctrl_out.armed ? "ARMED" : "STBY",
                     control_get_target(), (long)height_cm,
                     ctrl_out.elevon_left_deg, ctrl_out.elevon_right_deg);

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void app_main(void) {
  app_state_init();
  main_status_display_init();

  config_init();

  app_config_t cfg;
  config_load(&cfg);
  control_init(&cfg.control);

  /* Initialise CAN and create the 50 Hz task */
    if (can_init(on_can_rx) != ESP_OK) {
      ESP_LOGW(TAG, "CAN init failed — CAN TX disabled");
    }

    /* ── Servo hardware init and instance creation ── */
    (void)servo_drive_init_hw();
    s_servo_left  = servo_drive_open(SERVO_GPIO_LEFT);
    s_servo_right = servo_drive_open(SERVO_GPIO_RIGHT);
    app_state_set_servo(servo_drive_all_ready());

    if (s_servo_left != SERVO_CHANNEL_INVALID) {
      servo_drive_apply_cal(s_servo_left, &cfg.servo.channel[0]);
    }
    if (s_servo_right != SERVO_CHANNEL_INVALID) {
      servo_drive_apply_cal(s_servo_right, &cfg.servo.channel[1]);
    }
    if (servo_drive_any_cal_mode()) {
      ESP_LOGW(TAG, "cal mode active on open");
    }
    if (s_servo_left == SERVO_CHANNEL_INVALID && s_servo_right == SERVO_CHANNEL_INVALID) {
      ESP_LOGW(TAG, "All servos in simulated mode");
    }

  if (xTaskCreate(ctrl_task, "ctrl", 8192, NULL, 2, NULL) != pdPASS) {
    ESP_LOGE(TAG, "ctrl task create failed");
  }

  const bool imu_ok = (imu_init() == ESP_OK);
  app_state_set_imu(imu_ok);
  if (!imu_ok) {
    ESP_LOGW(TAG, "IMU off — no attitude CAN");
  }

  const bool height_ok = (height_init() == ESP_OK);
  app_state_set_height(height_ok);
  if (height_is_simulated()) {
    ESP_LOGW(TAG, "Height sensor in simulated mode");
  }

  webui_start();
}
