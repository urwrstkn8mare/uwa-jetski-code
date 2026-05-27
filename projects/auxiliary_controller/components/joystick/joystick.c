#include "joystick.h"

#include <stdint.h>

#include "can.h"
#include "can_ids.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "status_ui.h"

static const char *TAG = "joystick";

static adc_oneshot_unit_handle_t s_adc;
static adc_channel_t s_bank_ch;
static adc_channel_t s_pitch_ch;

static uint16_t raw_to_pct(int raw, int mn, int zero, int mx) {
    int lo = mn < mx ? mn : mx;
    int hi = mn < mx ? mx : mn;
    if (raw < lo) raw = lo;
    if (raw > hi) raw = hi;
    if (raw == zero) return 50u;

    bool lower_half = (mn < mx) ? (raw < zero) : (raw > zero);
    if (lower_half) {
        int span = zero - mn;
        if (span == 0) return 50u;
        return (uint16_t)(((int64_t)(raw - mn) * 50) / span);
    } else {
        int span = mx - zero;
        if (span == 0) return 50u;
        return (uint16_t)(50u + (uint16_t)(((int64_t)(raw - zero) * 50) / span));
    }
}

static void joystick_tx_task(void *arg) {
    (void)arg;
    for (;;) {
        int bank_raw = 0, pitch_raw = 0;
        adc_oneshot_read(s_adc, s_bank_ch, &bank_raw);
        adc_oneshot_read(s_adc, s_pitch_ch, &pitch_raw);

        uint16_t bank_pct  = raw_to_pct(bank_raw,  CONFIG_JOYSTICK_BANK_ADC_MIN,  CONFIG_JOYSTICK_BANK_ADC_ZERO,  CONFIG_JOYSTICK_BANK_ADC_MAX);
        uint16_t pitch_pct = raw_to_pct(pitch_raw, CONFIG_JOYSTICK_PITCH_ADC_MIN, CONFIG_JOYSTICK_PITCH_ADC_ZERO, CONFIG_JOYSTICK_PITCH_ADC_MAX);

        can_joystick_t joy = { .bank_pct = bank_pct, .pitch_pct = pitch_pct };
        (void)can_tx(CAN_ID_JOYSTICK, (const uint8_t *)&joy, sizeof(joy));

        status_ui_update("Joystick", "bank=%u%% pitch=%u%%", (unsigned)bank_pct, (unsigned)pitch_pct);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

esp_err_t joystick_init(void) {
    adc_unit_t bank_u, pitch_u;
    esp_err_t err = adc_oneshot_io_to_channel(CONFIG_JOYSTICK_BANK_GPIO, &bank_u, &s_bank_ch);
    ESP_RETURN_ON_ERROR(err, TAG, "Bank GPIO %d not a valid ADC pin", CONFIG_JOYSTICK_BANK_GPIO);

    err = adc_oneshot_io_to_channel(CONFIG_JOYSTICK_PITCH_GPIO, &pitch_u, &s_pitch_ch);
    ESP_RETURN_ON_ERROR(err, TAG, "Pitch GPIO %d not a valid ADC pin", CONFIG_JOYSTICK_PITCH_GPIO);

    if (bank_u != pitch_u) {
        ESP_LOGE(TAG, "Bank GPIO %d and pitch GPIO %d must be on the same ADC unit",
                 CONFIG_JOYSTICK_BANK_GPIO, CONFIG_JOYSTICK_PITCH_GPIO);
        return ESP_ERR_INVALID_ARG;
    }

    const adc_oneshot_unit_init_cfg_t acfg = { .unit_id = bank_u, .clk_src = ADC_DIGI_CLK_SRC_DEFAULT };
    err = adc_oneshot_new_unit(&acfg, &s_adc);
    ESP_RETURN_ON_ERROR(err, TAG, "adc_oneshot_new_unit failed");

    const adc_oneshot_chan_cfg_t chc = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12 };
    err = adc_oneshot_config_channel(s_adc, s_bank_ch, &chc);
    ESP_RETURN_ON_ERROR(err, TAG, "ADC bank channel config failed");
    err = adc_oneshot_config_channel(s_adc, s_pitch_ch, &chc);
    ESP_RETURN_ON_ERROR(err, TAG, "ADC pitch channel config failed");

    if (xTaskCreate(joystick_tx_task, "joystick_tx", 2048, NULL, 6, NULL) != pdPASS) {
        ESP_LOGE(TAG, "joystick_tx task create failed");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
