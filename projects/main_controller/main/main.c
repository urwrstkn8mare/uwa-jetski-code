#include "can.h"
#include "config.h"
#include "control.h"
#include "encoder_can.h"
#include "esp_log.h"
#include "height.h"
#include "imu.h"
#include "lvgl.h"
#include "servo_drive.h"
#include "status_ui.h"
#include "t_display_s3.h"
#include "webui.h"

static const char *TAG = "main";

static const int SERVO_GPIO_LEFT  = 10;
static const int SERVO_GPIO_RIGHT = 3;

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

  if (can_init() != ESP_OK) {
    ESP_LOGW(TAG, "CAN init failed — CAN TX disabled");
  }
  encoder_can_init(true);

  (void)servo_drive_init_hw();
  servo_channel_t servo_left  = servo_drive_open(SERVO_GPIO_LEFT);
  servo_channel_t servo_right = servo_drive_open(SERVO_GPIO_RIGHT);
  if (servo_left == SERVO_CHANNEL_INVALID && servo_right == SERVO_CHANNEL_INVALID) {
    ESP_LOGW(TAG, "All servos in simulated mode");
  }

  control_init(servo_left, servo_right);

  if (imu_init() != ESP_OK) {
    ESP_LOGW(TAG, "IMU off — no attitude CAN");
  }

  (void)height_init();

  webui_start();
}
