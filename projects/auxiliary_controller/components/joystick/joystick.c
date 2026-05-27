#include "joystick.h"

#include <stdint.h>
#include <string.h>

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

static uint16_t s_bank_pct;
static uint16_t s_pitch_pct;
static int      s_bank_raw;
static int      s_pitch_raw;

static uint16_t raw_to_pct(int raw, int mn, int zero, int mx) {
    int lo = mn < mx ? mn : mx;
    int hi = mn < mx ? mx : mn;
    if (raw < lo) raw = lo;
    if (raw > hi) raw = hi;
    if (raw == zero) return 50u;

    /* Determine which half based on travel direction (mn<mx = normal, mn>mx = inverted) */
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

static esp_err_t configure_channel(adc_channel_t ch) {
    const adc_oneshot_chan_cfg_t chc = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12,
    };
    return adc_oneshot_config_channel(s_adc, ch, &chc);
}

static void joystick_tx_task(void *arg) {
    (void)arg;
    uint32_t loop = 0;
    for (;;) {
        adc_channel_t bank_ch;
        adc_unit_t    bank_u;
        if (adc_oneshot_io_to_channel(CONFIG_JOYSTICK_BANK_GPIO, &bank_u, &bank_ch) == ESP_OK) {
            int raw = 0;
            if (adc_oneshot_read(s_adc, bank_ch, &raw) == ESP_OK) {
                s_bank_raw = raw;
                s_bank_pct = raw_to_pct(raw, CONFIG_JOYSTICK_BANK_ADC_MIN, CONFIG_JOYSTICK_BANK_ADC_ZERO, CONFIG_JOYSTICK_BANK_ADC_MAX);
            }
        }

        adc_channel_t pitch_ch;
        adc_unit_t    pitch_u;
        if (adc_oneshot_io_to_channel(CONFIG_JOYSTICK_PITCH_GPIO, &pitch_u, &pitch_ch) == ESP_OK) {
            int raw = 0;
            if (adc_oneshot_read(s_adc, pitch_ch, &raw) == ESP_OK) {
                s_pitch_raw = raw;
                s_pitch_pct = raw_to_pct(raw, CONFIG_JOYSTICK_PITCH_ADC_MIN, CONFIG_JOYSTICK_PITCH_ADC_ZERO, CONFIG_JOYSTICK_PITCH_ADC_MAX);
            }
        }

        can_joystick_t joy = {
            .bank_pct  = s_bank_pct,
            .pitch_pct = s_pitch_pct,
        };
        (void)can_tx(CAN_ID_JOYSTICK, (const uint8_t *)&joy, sizeof(joy));

        if ((loop % 20u) == 0u) {
            status_ui_update("Joystick", "bank=%u%% pitch=%u%% raw_b=%d raw_p=%d",
                             (unsigned)s_bank_pct, (unsigned)s_pitch_pct,
                             s_bank_raw, s_pitch_raw);
        }

        loop++;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


esp_err_t joystick_init(void) {
#if CONFIG_JOYSTICK_SKIP_HW
    ESP_LOGW(TAG, "Joystick disabled by Kconfig (CONFIG_JOYSTICK_SKIP_HW)");
    return ESP_FAIL;
#endif

    adc_channel_t bank_ch;
    adc_unit_t    bank_u;
    esp_err_t err = adc_oneshot_io_to_channel(CONFIG_JOYSTICK_BANK_GPIO, &bank_u, &bank_ch);
    ESP_RETURN_ON_ERROR(err, TAG, "Bank GPIO %d is not a valid ADC pin", CONFIG_JOYSTICK_BANK_GPIO);

    adc_channel_t pitch_ch;
    adc_unit_t    pitch_u;
    err = adc_oneshot_io_to_channel(CONFIG_JOYSTICK_PITCH_GPIO, &pitch_u, &pitch_ch);
    ESP_RETURN_ON_ERROR(err, TAG, "Pitch GPIO %d is not a valid ADC pin", CONFIG_JOYSTICK_PITCH_GPIO);

    if (bank_u != pitch_u) {
        ESP_LOGE(TAG, "Bank GPIO %d (unit %d) and pitch GPIO %d (unit %d) must be on the same ADC unit",
                 CONFIG_JOYSTICK_BANK_GPIO, (int)bank_u, CONFIG_JOYSTICK_PITCH_GPIO, (int)pitch_u);
        return ESP_ERR_INVALID_ARG;
    }

    const adc_oneshot_unit_init_cfg_t acfg = {
        .unit_id = bank_u,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
    };
    err = adc_oneshot_new_unit(&acfg, &s_adc);
    ESP_RETURN_ON_ERROR(err, TAG, "adc new unit failed");

    err = configure_channel(bank_ch);
    ESP_RETURN_ON_ERROR(err, TAG, "ADC bank channel config failed (GPIO %d)", CONFIG_JOYSTICK_BANK_GPIO);

    err = configure_channel(pitch_ch);
    ESP_RETURN_ON_ERROR(err, TAG, "ADC pitch channel config failed (GPIO %d)", CONFIG_JOYSTICK_PITCH_GPIO);

    ESP_LOGI(TAG, "Joystick ADC: bank=GPIO%d pitch=GPIO%d", CONFIG_JOYSTICK_BANK_GPIO, CONFIG_JOYSTICK_PITCH_GPIO);

    if (xTaskCreate(joystick_tx_task, "joystick_tx", 4096, NULL, 6, NULL) != pdPASS) {
        ESP_LOGE(TAG, "joystick_tx task create failed");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

uint16_t joystick_get_bank_pct(void)  { return s_bank_pct; }
uint16_t joystick_get_pitch_pct(void) { return s_pitch_pct; }
int      joystick_get_bank_raw(void)  { return s_bank_raw; }
int      joystick_get_pitch_raw(void) { return s_pitch_raw; }
