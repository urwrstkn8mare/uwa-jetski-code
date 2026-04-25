#include "imu.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

void app_main(void) {
    ESP_ERROR_CHECK(imu_init());
    ESP_LOGI(TAG, "IMU ready, reading pitch/roll...");

    for (;;) {
        float pitch, roll;
        if (imu_get_pitch_roll(&pitch, &roll) == ESP_OK) {
            ESP_LOGI(TAG, "Pitch: %6.2f deg  Roll: %6.2f deg", pitch, roll);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
