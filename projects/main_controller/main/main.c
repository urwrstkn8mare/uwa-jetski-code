#include "imu.h"
#include "can.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char *TAG = "main";

#define CAN_ID_ATTITUDE 0x100

void app_main(void) {
    ESP_ERROR_CHECK(imu_init());
    can_init(NULL);
    ESP_LOGI(TAG, "IMU and CAN ready, reading pitch/roll...");

    for (;;) {
        float pitch, roll;
        if (imu_get_pitch_roll(&pitch, &roll) == ESP_OK) {
            ESP_LOGI(TAG, "Pitch: %6.2f deg  Roll: %6.2f deg", pitch, roll);

            int16_t pitch_i = (int16_t)roundf(pitch);
            int16_t roll_i = (int16_t)roundf(roll);
            uint8_t can_data[4];
            memcpy(&can_data[0], &pitch_i, sizeof(pitch_i));
            memcpy(&can_data[2], &roll_i, sizeof(roll_i));
            can_tx(CAN_ID_ATTITUDE, can_data, sizeof(can_data));
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
