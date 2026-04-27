#include "height.h"

#include "a02yyuw.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "height";

static SemaphoreHandle_t s_mutex;
static int32_t s_height_cm = 0;
static bool s_initialized = false;

static void height_task(void *pvParameters) {
    (void)pvParameters;

    a02yyuw_dev_t dev = {0};

    /* The library hardcodes UART TX to NC, so we reuse the TX GPIO as the
     * sensor's mode-select pin.  On the A02YYUW, pulling RX high selects
     * processed (filtered) data; pulling it low selects real-time data. */
    esp_err_t ret = a02yyuw_init(&dev, CONFIG_HEIGHT_UART_PORT,
                                 CONFIG_HEIGHT_UART_RX_GPIO,
                                 CONFIG_HEIGHT_UART_TX_GPIO,
                                 A02YYUW_PIN_SELECT_PROCESSED);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "a02yyuw_init failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    /* Fix the library's incorrect flow-control setting */
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(CONFIG_HEIGHT_UART_PORT, &uart_config));

    ESP_LOGI(TAG, "A02YYUW ready on port %d RX GPIO %d mode-select GPIO %d (processed)",
             CONFIG_HEIGHT_UART_PORT, CONFIG_HEIGHT_UART_RX_GPIO, CONFIG_HEIGHT_UART_TX_GPIO);

    s_initialized = true;

    for (;;) {
        uint16_t distance_mm = 0;
        ret = a02yyuw_read_distance(&dev, &distance_mm);
        if (ret == ESP_OK) {
            int32_t height_cm = (int32_t)(distance_mm / 10);
            if (xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
                s_height_cm = height_cm;
                xSemaphoreGive(s_mutex);
            }
            ESP_LOGD(TAG, "Height: %ld cm (%u mm)", height_cm, distance_mm);
        } else {
            ESP_LOGD(TAG, "Read failed: %s", esp_err_to_name(ret));
        }
    }
}

esp_err_t height_init(void) {
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }

    xTaskCreate(height_task, "height_task", 4096, NULL, 1, NULL);

    while (!s_initialized) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    return ESP_OK;
}

esp_err_t height_get_cm(int32_t *height_cm) {
    if (!s_initialized) {
        return ESP_FAIL;
    }
    if (height_cm == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }
    *height_cm = s_height_cm;
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}
