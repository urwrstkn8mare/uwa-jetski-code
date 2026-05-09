#include "height.h"

#include <inttypes.h>
#include <stdio.h>

#include "a02yyuw.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "status_ui.h"

static const char *TAG = "height";

#ifndef CONFIG_HEIGHT_PROBE_TIMEOUT_MS
#define CONFIG_HEIGHT_PROBE_TIMEOUT_MS 2000
#endif

static SemaphoreHandle_t s_mutex;
static int32_t s_height_cm = 0;
static bool s_initialized = false;
static bool s_init_failed = false;
static volatile bool s_height_task_alive;

static void height_task(void *pvParameters) {
    (void)pvParameters;

    s_height_task_alive = true;

    a02yyuw_dev_t dev = {0};

    esp_err_t ret = a02yyuw_init(&dev, CONFIG_HEIGHT_UART_PORT,
                                 CONFIG_HEIGHT_UART_RX_GPIO,
                                 CONFIG_HEIGHT_UART_TX_GPIO,
                                 A02YYUW_PIN_SELECT_PROCESSED);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "a02yyuw_init failed: %s", esp_err_to_name(ret));
        s_height_task_alive = false;
        s_init_failed = true;
        status_ui_update("Height", "H: off (init failed)");
        vTaskDelete(NULL);
        return;
    }

    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ret = uart_param_config(CONFIG_HEIGHT_UART_PORT, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(ret));
        (void)a02yyuw_deinit(&dev);
        s_height_task_alive = false;
        s_init_failed = true;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "A02 probing %d ms (RX GPIO %d)…", CONFIG_HEIGHT_PROBE_TIMEOUT_MS,
             CONFIG_HEIGHT_UART_RX_GPIO);

    bool seen_valid = false;
    const int64_t t_deadline = esp_timer_get_time() + (int64_t)CONFIG_HEIGHT_PROBE_TIMEOUT_MS * 1000LL;
    uint16_t first_mm = 0;

    while (esp_timer_get_time() < t_deadline && !seen_valid) {
        uint16_t distance_mm = 0;
        ret = a02yyuw_read_distance(&dev, &distance_mm);
        if (ret == ESP_OK && distance_mm >= A02YYUW_MIN_RANGE && distance_mm <= A02YYUW_MAX_RANGE) {
            seen_valid = true;
            first_mm = distance_mm;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (!seen_valid) {
        ESP_LOGW(TAG, "Ultrasonic offline / no replies — freeing UART%d (disconnect is OK)",
                 CONFIG_HEIGHT_UART_PORT);
        (void)a02yyuw_deinit(&dev);
        s_height_task_alive = false;
        s_init_failed = true;
        status_ui_update("Height", "H: off (ultrasonic N/C)");
        vTaskDelete(NULL);
        return;
    }

    if (xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
        s_height_cm = (int32_t)(first_mm / 10);
        xSemaphoreGive(s_mutex);
    }

    ESP_LOGI(TAG, "A02YYUW OK on UART%d (%u mm) — streaming height", CONFIG_HEIGHT_UART_PORT,
             (unsigned)first_mm);

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
            status_ui_update("Height", "H:%" PRId32 " cm", height_cm);
            ESP_LOGD(TAG, "Height: %ld cm (%u mm)", height_cm, distance_mm);
        } else {
            ESP_LOGD(TAG, "Read failed: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

esp_err_t height_init(void) {
#if CONFIG_HEIGHT_SKIP_HW
    ESP_LOGW(TAG, "Height sensor disabled by Kconfig (CONFIG_HEIGHT_SKIP_HW)");
    return ESP_FAIL;
#endif

    static bool mutex_ready;

    if (!mutex_ready) {
        s_mutex = xSemaphoreCreateMutex();
        if (s_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return ESP_FAIL;
        }
        mutex_ready = true;
    }

    if (s_initialized) {
        return ESP_OK;
    }

    if (!s_height_task_alive) {
        s_init_failed = false;
        BaseType_t r = xTaskCreate(height_task, "height_task", 4096, NULL, 1, NULL);
        if (r != pdPASS) {
            ESP_LOGW(TAG, "height task create failed");
            return ESP_ERR_NO_MEM;
        }
    }

    const int max_wait = 45;
    int waited = 0;
    while (!s_initialized && !s_init_failed && waited < max_wait) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        waited++;
    }
    if (s_init_failed) {
        ESP_LOGW(TAG, "Height skipped — ultrasonic not detected or UART error");
        return ESP_FAIL;
    }
    if (!s_initialized) {
        ESP_LOGW(TAG, "Height init timed out (sensor task unusually slow)");
        return ESP_ERR_TIMEOUT;
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
