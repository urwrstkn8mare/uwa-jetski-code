#include "servo_drive.h"

#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include <stdio.h>

static const char *TAG = "servo_pwm";
static bool s_ready;
static bool s_simulated;
static uint32_t s_ch0_us = 1500;
static uint32_t s_ch1_us = 1500;

static int16_t pulse_to_deg(int32_t pulse_us) { return (int16_t)((pulse_us - 1500) / 25); }

static uint32_t clamp_pulse_us(uint32_t us) {
  if (us < 1300u) {
    return 1300u;
  }
  if (us > 1800u) {
    return 1800u;
  }
  return us;
}

static uint32_t pulse_us_to_duty(uint32_t pulse_us) {
  return (pulse_us * ((1u << LEDC_TIMER_14_BIT) - 1u)) / 20000u;
}

bool servo_drive_is_ready(void) { return s_ready; }

bool servo_drive_is_simulated(void) { return s_simulated; }

esp_err_t servo_drive_init(void) {
#if CONFIG_SERVO_SKIP_HW
  ESP_LOGW(TAG, "Servo PWM disabled by Kconfig — entering simulated mode");
  s_simulated = true;
  s_ready = true;
  status_ui_update("Servo", "SIM %" PRIu32 " %" PRIu32, s_ch0_us, s_ch1_us);
  return ESP_OK;
#endif

  ledc_timer_config_t timer = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_14_BIT,
      .timer_num = LEDC_TIMER_0,
      .freq_hz = 50,
      .clk_cfg = LEDC_AUTO_CLK,
  };
  esp_err_t e = ledc_timer_config(&timer);
  if (e != ESP_OK) {
    ESP_LOGW(TAG, "timer config failed (%s) — entering simulated mode", esp_err_to_name(e));
    s_simulated = true;
    s_ready = true;
    status_ui_update("Servo", "SIM %" PRIu32 " %" PRIu32, s_ch0_us, s_ch1_us);
    return ESP_OK;
  }

  ledc_channel_config_t ch0 = {
      .gpio_num = CONFIG_SERVO1_GPIO,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_0,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_0,
      .duty = pulse_us_to_duty(s_ch0_us),
      .hpoint = 0,
  };
  e = ledc_channel_config(&ch0);
  if (e != ESP_OK) {
    ESP_LOGW(TAG, "ch0 config failed (%s) — entering simulated mode", esp_err_to_name(e));
    s_simulated = true;
    s_ready = true;
    status_ui_update("Servo", "SIM %" PRIu32 " %" PRIu32, s_ch0_us, s_ch1_us);
    return ESP_OK;
  }

  ledc_channel_config_t ch1 = {
      .gpio_num = CONFIG_SERVO2_GPIO,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_1,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_0,
      .duty = pulse_us_to_duty(s_ch1_us),
      .hpoint = 0,
  };
  e = ledc_channel_config(&ch1);
  if (e != ESP_OK) {
    ESP_LOGW(TAG, "ch1 config failed (%s) — entering simulated mode", esp_err_to_name(e));
    s_simulated = true;
    s_ready = true;
    status_ui_update("Servo", "SIM %" PRIu32 " %" PRIu32, s_ch0_us, s_ch1_us);
    return ESP_OK;
  }

  s_ready = true;
  status_ui_update("Servo", "%s %" PRIu32 " %" PRIu32, "HW", s_ch0_us, s_ch1_us);
  ESP_LOGI(TAG, "PWM servo outputs enabled: ch0->GPIO%d ch1->GPIO%d @ 50Hz",
           CONFIG_SERVO1_GPIO, CONFIG_SERVO2_GPIO);
  return ESP_OK;
}

void servo_drive_set_channels(uint32_t ch0_us, uint32_t ch1_us) {
  ch0_us = clamp_pulse_us(ch0_us);
  ch1_us = clamp_pulse_us(ch1_us);
  s_ch0_us = ch0_us;
  s_ch1_us = ch1_us;
  if (!s_ready) {
    return;
  }
  if (s_simulated) {
    status_ui_update("Servo", "SIM %" PRIu32 " %" PRIu32, ch0_us, ch1_us);
    return;
  }
  uint32_t d0 = pulse_us_to_duty(ch0_us);
  uint32_t d1 = pulse_us_to_duty(ch1_us);
  (void)ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, d0);
  (void)ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  (void)ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, d1);
  (void)ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
  status_ui_update("Servo", "HW %" PRIu32 " %" PRIu32, ch0_us, ch1_us);
}

void servo_drive_get_commanded_deg(int16_t *deg_a, int16_t *deg_b) {
  int16_t d0 = pulse_to_deg((int32_t)s_ch0_us);
  int16_t d1 = pulse_to_deg((int32_t)s_ch1_us);
  if (deg_a) {
    *deg_a = d0;
  }
  if (deg_b) {
    *deg_b = d1;
  }
}

void servo_drive_get_pulse_us(uint32_t *ch0_us_out, uint32_t *ch1_us_out) {
  if (ch0_us_out) {
    *ch0_us_out = s_ch0_us;
  }
  if (ch1_us_out) {
    *ch1_us_out = s_ch1_us;
  }
}
