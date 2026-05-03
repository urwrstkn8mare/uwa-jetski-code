#include "rudder_pot.h"

#include <stdint.h>

#include "can.h"
#include "can_ids.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "rudder_pot";

static adc_oneshot_unit_handle_t s_adc;
static uint16_t s_last_pct;

static void pot_tx_task(void *arg) {
  (void)arg;
  for (;;) {
    if (s_adc != NULL) {
      int raw = 0;
      adc_channel_t ch;
      adc_unit_t u;
      if (adc_oneshot_io_to_channel(CONFIG_POT_GPIO_NUM, &u, &ch) == ESP_OK) {
        if (adc_oneshot_read(s_adc, ch, &raw) == ESP_OK) {
          int mn = CONFIG_POT_ADC_RAW_MIN;
          int mx = CONFIG_POT_ADC_RAW_MAX;
          int v = raw;
          if (v < mn) {
            v = mn;
          }
          if (v > mx) {
            v = mx;
          }
          uint16_t pct = 0;
          if (mx > mn) {
            pct = (uint16_t)(((int64_t)(v - mn) * 100) / (mx - mn));
          }
          s_last_pct = pct;
          uint8_t b[2];
          memcpy(b, &pct, sizeof(pct));
          can_tx(CAN_ID_POTENTIOMETER, b, sizeof(b));
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

esp_err_t rudder_pot_init(void) {
  adc_oneshot_unit_init_cfg_t acfg = {
      .unit_id = (CONFIG_POT_ADC_UNIT == 2) ? ADC_UNIT_2 : ADC_UNIT_1,
      .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
  };
  esp_err_t err = adc_oneshot_new_unit(&acfg, &s_adc);
  if (err != ESP_OK) {
    return err;
  }
  adc_oneshot_chan_cfg_t chc = {
      .bitwidth = ADC_BITWIDTH_DEFAULT,
      .atten = ADC_ATTEN_DB_12,
  };
  adc_channel_t ch;
  adc_unit_t uu;
  if (adc_oneshot_io_to_channel(CONFIG_POT_GPIO_NUM, &uu, &ch) != ESP_OK ||
      adc_oneshot_config_channel(s_adc, ch, &chc) != ESP_OK) {
    ESP_LOGW(TAG, "ADC channel config failed on GPIO %d", CONFIG_POT_GPIO_NUM);
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "ADC rudder on GPIO %d", CONFIG_POT_GPIO_NUM);
  xTaskCreate(pot_tx_task, "pot_tx", 4096, NULL, 6, NULL);
  return ESP_OK;
}

uint16_t rudder_pot_get_last_pct(void) { return s_last_pct; }
