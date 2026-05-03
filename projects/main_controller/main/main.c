#include "can.h"
#include "can_ids.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "height.h"
#include "imu.h"
#include "lvgl.h"
#include "t_display_s3.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "main";

static SemaphoreHandle_t s_iu_mux;
static uint16_t s_pot_pct;
static int32_t s_lat_e7;
static int32_t s_lon_e7;
static int16_t s_speed_kmh_x10;
static int16_t s_heading_cdeg;
static bool s_have_gps;

static tdisplays3_handle_t s_board;
static lv_obj_t *s_dbg_label;

#define SERVO_TIMER ((ledc_timer_t)CONFIG_SERVO_LEDC_TIMER_IDX)
#define CH_A ((ledc_channel_t)CONFIG_SERVO_LEDC_CHANNEL_A)
#define CH_B ((ledc_channel_t)CONFIG_SERVO_LEDC_CHANNEL_B)

static void can_rx_cb(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp) {
  (void)timestamp;
  if (buffer == NULL || s_iu_mux == NULL) {
    return;
  }
  if (xSemaphoreTake(s_iu_mux, pdMS_TO_TICKS(20)) != pdTRUE) {
    return;
  }
  switch (header_id) {
  case CAN_ID_POTENTIOMETER: {
    uint16_t v;
    memcpy(&v, buffer, sizeof(v));
    s_pot_pct = (v > 100) ? 100 : v;
    break;
  }
  case CAN_ID_GPS_POSITION:
    memcpy(&s_lat_e7, &buffer[0], sizeof(s_lat_e7));
    memcpy(&s_lon_e7, &buffer[4], sizeof(s_lon_e7));
    s_have_gps = true;
    break;
  case CAN_ID_GPS_VELOCITY:
    memcpy(&s_speed_kmh_x10, &buffer[0], sizeof(s_speed_kmh_x10));
    memcpy(&s_heading_cdeg, &buffer[2], sizeof(s_heading_cdeg));
    break;
  default:
    break;
  }
  xSemaphoreGive(s_iu_mux);
}

static void ledc_servo_set_us(ledc_channel_t ch, uint32_t pulse_us) {
  if (pulse_us < 500) {
    pulse_us = 500;
  }
  if (pulse_us > 2500) {
    pulse_us = 2500;
  }
  const uint32_t duty = (pulse_us * 8192) / 20000;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

static int16_t pulse_to_deg(int32_t pulse_us) {
  return (int16_t)((pulse_us - 1500) / 25);
}

static void control_task(void *arg) {
  (void)arg;
  uint32_t loop = 0;
  for (;;) {
    float pitch, roll;
    if (imu_get_pitch_roll(&pitch, &roll) == ESP_OK) {
      int16_t pitch_i = (int16_t)lroundf(pitch);
      int16_t roll_i = (int16_t)lroundf(roll);
      uint8_t b[4];
      memcpy(&b[0], &pitch_i, sizeof(pitch_i));
      memcpy(&b[2], &roll_i, sizeof(roll_i));
      can_tx(CAN_ID_ATTITUDE, b, sizeof(b));
    }

    int32_t hcm;
    if (height_get_cm(&hcm) == ESP_OK) {
      uint16_t hu = (uint16_t)hcm;
      uint8_t b[2];
      memcpy(b, &hu, sizeof(hu));
      can_tx(CAN_ID_HEIGHT, b, sizeof(b));
    }

    uint16_t pot = 0;
    if (s_iu_mux && xSemaphoreTake(s_iu_mux, pdMS_TO_TICKS(20)) == pdTRUE) {
      pot = s_pot_pct;
      xSemaphoreGive(s_iu_mux);
    }
    if (pot > 100) {
      pot = 100;
    }
    const uint32_t pulse = (uint32_t)(1000 + (int32_t)pot * 10);
    ledc_servo_set_us(CH_A, pulse);
    ledc_servo_set_us(CH_B, pulse);

    int16_t sa = pulse_to_deg((int32_t)pulse);
    int16_t sb = sa;
    uint8_t sb_c[4];
    memcpy(&sb_c[0], &sa, sizeof(sa));
    memcpy(&sb_c[2], &sb, sizeof(sb));
    can_tx(CAN_ID_SERVO_POS, sb_c, sizeof(sb_c));

    loop++;
    if (loop % 200 == 0) {
      uint32_t a, f;
      can_get_tx_stats(&a, &f);
      if (f > 0) {
        ESP_LOGW(TAG, "CAN TX %" PRIu32 " tries %" PRIu32 " fail", a, f);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

static void ui_refresh_timer(lv_timer_t *t) {
  (void)t;
  float pitch = 0, roll = 0;
  (void)imu_get_pitch_roll(&pitch, &roll);
  int32_t hcm = -1;
  (void)height_get_cm(&hcm);

  uint16_t pot = 0;
  int32_t lat = 0, lon = 0;
  int speed_x10 = 0;
  int hdg_c = 0;
  bool gps = false;
  if (s_iu_mux && xSemaphoreTake(s_iu_mux, pdMS_TO_TICKS(10)) == pdTRUE) {
    pot = s_pot_pct;
    lat = s_lat_e7;
    lon = s_lon_e7;
    speed_x10 = s_speed_kmh_x10;
    hdg_c = s_heading_cdeg;
    gps = s_have_gps;
    xSemaphoreGive(s_iu_mux);
  }

  char buf[512];
  snprintf(buf, sizeof(buf),
           "MAIN node\n"
           "P:%.1f R:%.1f deg\n"
           "H:%" PRId32 " cm\n"
           "Pot:%u -> %lu us\n"
           "lat_e7:%" PRId32 " lon_e7:%" PRId32 "\n"
           "spd:%d.%d km/h hdg:%d.%02d deg\n"
           "%s",
           (double)pitch, (double)roll, hcm, (unsigned)pot,
           (unsigned long)(1000u + (unsigned)pot * 10u), lat, lon, speed_x10 / 10,
           abs(speed_x10 % 10), hdg_c / 100, abs(hdg_c % 100),
           gps ? "GPS ok" : "no GPS fix pos");

  if (tdisplays3_display_lock(100)) {
    lv_label_set_text(s_dbg_label, buf);
    tdisplays3_display_unlock();
  }
}

static esp_err_t init_retry(const char *name, esp_err_t (*fn)(void), int n) {
  esp_err_t r = ESP_FAIL;
  for (int i = 0; i < n; i++) {
    r = fn();
    if (r == ESP_OK) {
      return ESP_OK;
    }
    ESP_LOGW(TAG, "%s init fail (%d/%d): %s", name, i + 1, n, esp_err_to_name(r));
    vTaskDelay(pdMS_TO_TICKS(400));
  }
  return r;
}

static esp_err_t can_wrap(void) { return can_init(can_rx_cb); }

void app_main(void) {
  s_iu_mux = xSemaphoreCreateMutex();

  ESP_ERROR_CHECK(tdisplays3_init(&s_board));
  if (!tdisplays3_display_lock(200)) {
    ESP_LOGE(TAG, "Display lock failed");
    return;
  }
  s_dbg_label = lv_label_create(lv_screen_active());
  lv_obj_set_width(s_dbg_label, lv_display_get_horizontal_resolution(NULL) - 8);
  lv_label_set_long_mode(s_dbg_label, LV_LABEL_LONG_WRAP);
  lv_obj_align(s_dbg_label, LV_ALIGN_TOP_LEFT, 4, 4);
  lv_label_set_text(s_dbg_label, "MAIN\ninit…");
  tdisplays3_display_unlock();
  (void)lv_timer_create(ui_refresh_timer, 200, NULL);

  if (init_retry("IMU", imu_init, 3) != ESP_OK) {
    ESP_LOGW(TAG, "IMU not available");
  }
  if (init_retry("Height", height_init, 3) != ESP_OK) {
    ESP_LOGW(TAG, "Height sensor not available");
  }
  if (init_retry("CAN", can_wrap, 3) != ESP_OK) {
    ESP_LOGE(TAG, "CAN required");
    return;
  }

  const ledc_timer_config_t tcfg = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .timer_num = SERVO_TIMER,
      .duty_resolution = LEDC_TIMER_13_BIT,
      .freq_hz = 50,
      .clk_cfg = LEDC_AUTO_CLK,
  };
  ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

  const ledc_channel_t chs[2] = {CH_A, CH_B};
  const int gpios[2] = {CONFIG_SERVO_A_GPIO, CONFIG_SERVO_B_GPIO};
  for (int i = 0; i < 2; i++) {
    ledc_channel_config_t c = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = chs[i],
        .timer_sel = SERVO_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = gpios[i],
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&c));
  }
  ESP_LOGI(TAG, "Servos LEDC ch A=%d B=%d", CONFIG_SERVO_A_GPIO, CONFIG_SERVO_B_GPIO);

  xTaskCreate(control_task, "control", 4096, NULL, 6, NULL);
}
