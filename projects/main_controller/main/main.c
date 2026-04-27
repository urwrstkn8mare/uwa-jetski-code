#include "imu.h"
#include "height.h"
#include "can.h"
#include "can_ids.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include <string.h>
#include <math.h>

static const char *TAG = "main";

#define SERVO_GPIO GPIO_NUM_1
#define SERVO_LEDC_CHANNEL LEDC_CHANNEL_0
#define SERVO_LEDC_TIMER LEDC_TIMER_0

static SemaphoreHandle_t s_pot_mutex;
static uint16_t s_potentiometer_value = 0;
static uint32_t s_loop_count = 0;

static void can_rx_cb(const uint8_t buffer[8], uint32_t header_id, uint64_t timestamp)
{
    (void)timestamp;
    if (buffer == NULL) {
        return;
    }
    if (header_id == CAN_ID_POTENTIOMETER) {
        uint16_t pot_val;
        memcpy(&pot_val, &buffer[0], sizeof(pot_val));
        if (xSemaphoreTake(s_pot_mutex, portMAX_DELAY) == pdTRUE) {
            s_potentiometer_value = pot_val;
            xSemaphoreGive(s_pot_mutex);
        }
        ESP_LOGI(TAG, "Potentiometer: %u", pot_val);
    }
}

static void control_task(void *arg)
{
    (void)arg;
    for (;;) {
        float pitch, roll;
        if (imu_get_pitch_roll(&pitch, &roll) == ESP_OK) {
            int16_t pitch_i = (int16_t)roundf(pitch);
            int16_t roll_i = (int16_t)roundf(roll);
            uint8_t can_data[4];
            memcpy(&can_data[0], &pitch_i, sizeof(pitch_i));
            memcpy(&can_data[2], &roll_i, sizeof(roll_i));
            can_tx(CAN_ID_ATTITUDE, can_data, sizeof(can_data));
        }

        int32_t height_cm;
        if (height_get_cm(&height_cm) == ESP_OK) {
            uint16_t height_u = (uint16_t)height_cm;
            uint8_t can_data[2];
            memcpy(&can_data[0], &height_u, sizeof(height_u));
            can_tx(CAN_ID_HEIGHT, can_data, sizeof(can_data));
        }

        // Update servo based on potentiometer (0-100 -> 1000-2000 µs)
        uint16_t pot_val = 0;
        if (xSemaphoreTake(s_pot_mutex, portMAX_DELAY) == pdTRUE) {
            pot_val = s_potentiometer_value;
            xSemaphoreGive(s_pot_mutex);
        }
        if (pot_val > 100) pot_val = 100;
        uint32_t pulse_us = 1000 + (pot_val * 10);  // 1000-2000 µs
        uint32_t duty = (pulse_us * 8192) / 20000;  // Convert to duty cycle (13-bit = 8192 max)
        ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_LEDC_CHANNEL, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_LEDC_CHANNEL);

        // Transmit servo position (-20..20 degrees)
        int16_t servo_deg = (int16_t)(((int32_t)pulse_us - 1500) / 25);
        uint8_t servo_data[2];
        memcpy(&servo_data[0], &servo_deg, sizeof(servo_deg));
        can_tx(CAN_ID_SERVO_POS, servo_data, sizeof(servo_data));

        // Periodically log CAN TX stats (every 10 seconds)
        s_loop_count++;
        if (s_loop_count % 200 == 0) {  // 200 × 50ms = 10 seconds
            uint32_t attempts, failures;
            can_get_tx_stats(&attempts, &failures);
            if (failures > 0) {
                ESP_LOGW(TAG, "CAN TX stats: %u attempts, %u failures (%.1f%% success)",
                         attempts, failures,
                         100.0f * (attempts - failures) / attempts);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static esp_err_t init_with_retry(const char *name, esp_err_t (*init_fn)(void), int retries)
{
    esp_err_t ret = ESP_FAIL;
    for (int i = 0; i < retries; i++) {
        ret = init_fn();
        if (ret == ESP_OK) {
            return ESP_OK;
        }
        ESP_LOGW(TAG, "%s init failed (attempt %d/%d): %s", name, i + 1, retries, esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGE(TAG, "%s init failed after %d attempts", name, retries);
    return ret;
}

static esp_err_t can_init_wrapper(void)
{
    return can_init(can_rx_cb);
}

void app_main(void)
{
    s_pot_mutex = xSemaphoreCreateMutex();
    if (s_pot_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create potentiometer mutex");
        return;
    }

    if (init_with_retry("IMU", imu_init, 3) != ESP_OK) {
        ESP_LOGW(TAG, "Continuing without IMU");
    }
    if (init_with_retry("Height", height_init, 3) != ESP_OK) {
        ESP_LOGW(TAG, "Continuing without height sensor");
    }
    if (init_with_retry("CAN", can_init_wrapper, 3) != ESP_OK) {
        ESP_LOGE(TAG, "CAN init failed, cannot continue");
        return;
    }
    ESP_LOGI(TAG, "IMU, height sensor and CAN ready");

    // Initialize LEDC for servo control
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = SERVO_LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 50,  // 50 Hz for servo (20 ms period)
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(ret));
        return;
    }

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = SERVO_LEDC_CHANNEL,
        .timer_sel = SERVO_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = SERVO_GPIO,
        .duty = 0,
        .hpoint = 0,
    };
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Servo PWM initialized on GPIO %d", SERVO_GPIO);

    xTaskCreate(control_task, "control", 4096, NULL, 6, NULL);
}
