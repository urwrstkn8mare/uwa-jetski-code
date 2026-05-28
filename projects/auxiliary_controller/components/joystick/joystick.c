#include "joystick.h"

#include <stdint.h>
#include <stdlib.h>

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
static adc_channel_t s_x_ch;
static adc_channel_t s_y_ch;

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
        int x_raw = 0, y_raw = 0;
        adc_oneshot_read(s_adc, s_x_ch, &x_raw);
        adc_oneshot_read(s_adc, s_y_ch, &y_raw);

        uint16_t x_pct = raw_to_pct(x_raw, CONFIG_JOYSTICK_X_ADC_MIN, CONFIG_JOYSTICK_X_ADC_ZERO, CONFIG_JOYSTICK_X_ADC_MAX);
        uint16_t y_pct = raw_to_pct(y_raw, CONFIG_JOYSTICK_Y_ADC_MIN, CONFIG_JOYSTICK_Y_ADC_ZERO, CONFIG_JOYSTICK_Y_ADC_MAX);

        int x_dev = (int)x_pct - 50;
        int y_dev = (int)y_pct - 50;
        if (abs(x_dev) >= abs(y_dev)) {
            y_pct = 50;
        } else {
            x_pct = 50;
        }

        can_joystick_t joy = { .x_pct = x_pct, .y_pct = y_pct };
        (void)can_tx(CAN_ID_JOYSTICK, (const uint8_t *)&joy, sizeof(joy));

        status_ui_update("Joystick", "x=%u%% y=%u%%", (unsigned)x_pct, (unsigned)y_pct);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

esp_err_t joystick_init(void) {
    adc_unit_t x_u, y_u;
    esp_err_t err = adc_oneshot_io_to_channel(CONFIG_JOYSTICK_X_GPIO, &x_u, &s_x_ch);
    ESP_RETURN_ON_ERROR(err, TAG, "X GPIO %d not a valid ADC pin", CONFIG_JOYSTICK_X_GPIO);

    err = adc_oneshot_io_to_channel(CONFIG_JOYSTICK_Y_GPIO, &y_u, &s_y_ch);
    ESP_RETURN_ON_ERROR(err, TAG, "Y GPIO %d not a valid ADC pin", CONFIG_JOYSTICK_Y_GPIO);

    if (x_u != y_u) {
        ESP_LOGE(TAG, "X GPIO %d and Y GPIO %d must be on the same ADC unit",
                 CONFIG_JOYSTICK_X_GPIO, CONFIG_JOYSTICK_Y_GPIO);
        return ESP_ERR_INVALID_ARG;
    }

    const adc_oneshot_unit_init_cfg_t acfg = { .unit_id = x_u, .clk_src = ADC_DIGI_CLK_SRC_DEFAULT };
    err = adc_oneshot_new_unit(&acfg, &s_adc);
    ESP_RETURN_ON_ERROR(err, TAG, "adc_oneshot_new_unit failed");

    const adc_oneshot_chan_cfg_t chc = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12 };
    err = adc_oneshot_config_channel(s_adc, s_x_ch, &chc);
    ESP_RETURN_ON_ERROR(err, TAG, "ADC X channel config failed");
    err = adc_oneshot_config_channel(s_adc, s_y_ch, &chc);
    ESP_RETURN_ON_ERROR(err, TAG, "ADC Y channel config failed");

    if (xTaskCreate(joystick_tx_task, "joystick_tx", 2048, NULL, 6, NULL) != pdPASS) {
        ESP_LOGE(TAG, "joystick_tx task create failed");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
