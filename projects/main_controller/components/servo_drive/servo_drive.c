#include "servo_drive.h"

#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "servo_drive";

static uint32_t s_last_pulse_us = 1500;

#define SERVO_TIMER ((ledc_timer_t)CONFIG_SERVO_LEDC_TIMER_IDX)
#define CH_A ((ledc_channel_t)CONFIG_SERVO_LEDC_CHANNEL_A)
#define CH_B ((ledc_channel_t)CONFIG_SERVO_LEDC_CHANNEL_B)

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

static int16_t pulse_to_deg(int32_t pulse_us) { return (int16_t)((pulse_us - 1500) / 25); }

esp_err_t servo_drive_init(void) {
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
  return ESP_OK;
}

void servo_drive_set_pct(uint16_t pot_pct_0_100) {
  uint16_t pot = pot_pct_0_100;
  if (pot > 100) {
    pot = 100;
  }
  s_last_pulse_us = (uint32_t)(1000 + (int32_t)pot * 10);
  ledc_servo_set_us(CH_A, s_last_pulse_us);
  ledc_servo_set_us(CH_B, s_last_pulse_us);
}

void servo_drive_get_commanded_deg(int16_t *deg_a, int16_t *deg_b) {
  int16_t d = pulse_to_deg((int32_t)s_last_pulse_us);
  if (deg_a) {
    *deg_a = d;
  }
  if (deg_b) {
    *deg_b = d;
  }
}
