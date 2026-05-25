#include "can.h"
#include "config.h"
#include "control.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "height.h"
#include "imu.h"
#include "lvgl.h"
#include "rudder.h"
#include "servo_drive.h"
#include "status_ui.h"
#include "t_display_s3.h"
#include "webui.h"

static const auto TAG = "main";

static servo_channel_t s_servo_left;
static servo_channel_t s_servo_right;
static const int SERVO_GPIO_LEFT  = 1;
static const int SERVO_GPIO_RIGHT = 2;

static TaskHandle_t s_ctrl_task_handle;

static void on_can_rx(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp) {
  (void)timestamp;
  rudder_on_can_rx(buffer, header_id);
}

static void on_control_armed(void) {
  if (s_ctrl_task_handle) {
    xTaskNotifyGive(s_ctrl_task_handle);
  }
}

static void on_can_status(const char *line) {
  status_ui_update("CAN", "%s", line);
}

/* ── 50 Hz stabilisation task — only runs while armed ── */
static void ctrl_task(void *arg) {
  (void)arg;
  s_ctrl_task_handle = xTaskGetCurrentTaskHandle();
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    while (control_is_armed() && !servo_drive_any_cal_mode()) {
      uint16_t pot_pct = 50;
      rudder_is_fresh(500, &pot_pct);

      float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
      if (imu_is_ready()) {
        imu_get_pitch_roll_yaw(&pitch, &roll, &yaw);
      }

      int32_t height_cm = 30;
      height_get_cm(&height_cm);

      control_output_t out;
      control_update((int16_t)height_cm, pitch, roll, pot_pct, &out);

      if (s_servo_left != SERVO_CHANNEL_INVALID) {
        servo_drive_set_degrees(s_servo_left, out.elevon_left_deg);
      }
      if (s_servo_right != SERVO_CHANNEL_INVALID) {
        servo_drive_set_degrees(s_servo_right, out.elevon_right_deg);
      }

      vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (servo_drive_any_cal_mode() && control_is_armed()) {
      control_disarm();
    }
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
  control_register_arm_cb(on_control_armed);

  if (can_init(on_can_rx) != ESP_OK) {
    ESP_LOGW(TAG, "CAN init failed — CAN TX disabled");
  }
  can_register_status_cb(on_can_status);

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
