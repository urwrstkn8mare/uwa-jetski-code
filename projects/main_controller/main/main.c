#include "imu.h"
#include "height.h"
#include "can.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include <string.h>
#include <math.h>

static const char *TAG = "main";

#define CAN_ID_ATTITUDE    0x100
#define CAN_ID_HEIGHT      0x101
#define CAN_ID_POTENTIOMETER 0x102

#define SERVO_GPIO GPIO_NUM_1
#define SERVO_LEDC_CHANNEL LEDC_CHANNEL_0
#define SERVO_LEDC_TIMER LEDC_TIMER_0

static volatile uint16_t s_potentiometer_value = 0;

static bool can_rx_cb(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp)
{
    (void)timestamp;
    if (buffer == NULL) {
        return false;
    }
    if (header_id == CAN_ID_POTENTIOMETER) {
        uint16_t pot_val;
        memcpy(&pot_val, &buffer[0], sizeof(pot_val));
        s_potentiometer_value = pot_val;
        ESP_LOGI(TAG, "Potentiometer: %u", pot_val);
    }
    return false;
}

void app_main(void) {
    ESP_ERROR_CHECK(imu_init());
    ESP_ERROR_CHECK(height_init());
    can_init(can_rx_cb);
    ESP_LOGI(TAG, "IMU, height sensor and CAN ready");

    // Initialize LEDC for servo control
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = SERVO_LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 50,  // 50 Hz for servo (20 ms period)
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = SERVO_LEDC_CHANNEL,
        .timer_sel = SERVO_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = SERVO_GPIO,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_LOGI(TAG, "Servo PWM initialized on GPIO %d", SERVO_GPIO);

    for (;;) {
        float pitch, roll;
        if (imu_get_pitch_roll(&pitch, &roll) == ESP_OK) {
            // ESP_LOGI(TAG, "Pitch: %6.2f deg  Roll: %6.2f deg", pitch, roll);

            int16_t pitch_i = (int16_t)roundf(pitch);
            int16_t roll_i = (int16_t)roundf(roll);
            uint8_t can_data[4];
            memcpy(&can_data[0], &pitch_i, sizeof(pitch_i));
            memcpy(&can_data[2], &roll_i, sizeof(roll_i));
            can_tx(CAN_ID_ATTITUDE, can_data, sizeof(can_data));
        }

        int32_t height_cm;
        if (height_get_cm(&height_cm) == ESP_OK) {
            // ESP_LOGI(TAG, "Height: %ld cm", height_cm);

            uint16_t height_u = (uint16_t)height_cm;
            uint8_t can_data[2];
            memcpy(&can_data[0], &height_u, sizeof(height_u));
            can_tx(CAN_ID_HEIGHT, can_data, sizeof(can_data));
        }

        // Update servo based on potentiometer (0-100 -> 1000-2000 µs)
        // At 50 Hz with 13-bit resolution: period = 2^13 / 50 = 163.84
        // 1000 µs = 163.84 * (1000 / 20000) = 8.192
        // 2000 µs = 163.84 * (2000 / 20000) = 16.384
        uint16_t pot_val = s_potentiometer_value;
        if (pot_val > 100) pot_val = 100;
        uint32_t pulse_us = 1000 + (pot_val * 10);  // 1000-2000 µs
        uint32_t duty = (pulse_us * 8192) / 20000;  // Convert to duty cycle (13-bit = 8192 max)
        ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_LEDC_CHANNEL, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_LEDC_CHANNEL);

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
