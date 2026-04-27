#include "height.h"

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define UART_RX_BUF_SIZE 128
#define FRAME_SIZE 4
#define FRAME_HEADER 0xFF

static const char *TAG = "height";

static SemaphoreHandle_t s_mutex;
static int32_t s_height_cm = 0;
static bool s_initialized = false;

static inline bool parse_frame(const uint8_t buf[FRAME_SIZE], uint16_t *distance_mm) {
    if (buf[0] != FRAME_HEADER) {
        return false;
    }
    uint8_t checksum = (buf[0] + buf[1] + buf[2]) & 0xFF;
    if (buf[3] != checksum) {
        return false;
    }
    *distance_mm = ((uint16_t)buf[1] << 8) | buf[2];
    return true;
}

static void height_task(void *pvParameters) {
    (void)pvParameters;

    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(CONFIG_HEIGHT_UART_PORT, UART_RX_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CONFIG_HEIGHT_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(CONFIG_HEIGHT_UART_PORT, CONFIG_HEIGHT_UART_TX_GPIO, CONFIG_HEIGHT_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "A02YYUW UART ready on port %d RX GPIO %d TX GPIO %d",
             CONFIG_HEIGHT_UART_PORT, CONFIG_HEIGHT_UART_RX_GPIO, CONFIG_HEIGHT_UART_TX_GPIO);

    /* Flush any stale data from the UART buffer */
    uart_flush_input(CONFIG_HEIGHT_UART_PORT);

    uint8_t rx_buf[FRAME_SIZE];
    int rx_idx = 0;
    s_initialized = true;

    for (;;) {
        uint8_t byte;
        int len = uart_read_bytes(CONFIG_HEIGHT_UART_PORT, &byte, 1, pdMS_TO_TICKS(10));
        if (len <= 0) {
            continue;
        }

        if (rx_idx == 0 && byte != FRAME_HEADER) {
            continue;
        }

        rx_buf[rx_idx++] = byte;

        if (rx_idx >= FRAME_SIZE) {
            rx_idx = 0;
            uint16_t distance_mm;
            if (parse_frame(rx_buf, &distance_mm)) {
                int32_t height_cm = (int32_t)(distance_mm / 10);
                if (xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
                    s_height_cm = height_cm;
                    xSemaphoreGive(s_mutex);
                }
                ESP_LOGD(TAG, "Height: %ld cm (%u mm)", height_cm, distance_mm);
            } else {
                ESP_LOGW(TAG, "Invalid frame, discarding");
            }
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
