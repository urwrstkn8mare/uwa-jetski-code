#include "FreeRTOSConfig.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "madgwick_filter.hpp"
#include <cmath>
extern "C" {
#include <mpu9250.h>
}

#define SCL_PIN 39
#define SDA_PIN 38

static const char *TAG = "mpu_example";

static const float ACCEL_SENSITIVITY = 16384.0f;
static const float GYRO_SENSITIVITY = 131.0f;
static const float MAG_BASE_SENSITIVITY = 0.15f;
static const float DEG_TO_RAD = M_PI / 180.0f;
static const int GYRO_CALIBRATION_SAMPLES = 500;
static const float MAG_OFFSET_X = -17.96f;
static const float MAG_OFFSET_Y = -14.92f;
static const float MAG_OFFSET_Z = -0.09f;
static const float MAG_SCALE_X = 0.972f;
static const float MAG_SCALE_Y = 1.003f;
static const float MAG_SCALE_Z = 1.026f;
static const bool MAG_CALIBRATE = (MAG_OFFSET_X == 0.0f && MAG_OFFSET_Y == 0.0f && MAG_OFFSET_Z == 0.0f);

void vImuTask(void *pvParameters) {
  i2c_master_bus_config_t bus_config = {};
  bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_config.i2c_port = I2C_NUM_0;
  bus_config.scl_io_num = static_cast<gpio_num_t>(SCL_PIN);
  bus_config.sda_io_num = static_cast<gpio_num_t>(SDA_PIN);
  bus_config.glitch_ignore_cnt = 7;
  bus_config.flags.enable_internal_pullup = true;
  i2c_master_bus_handle_t bus_handle;
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

  mpu9250_t mpu;
  mpu9250_config_t mpu_config = {};
  mpu_config.gyro_enabled = 1;
  mpu_config.accel_enabled = 1;
  mpu_config.temp_enabled = 1;
  mpu_config.mag_enabled = 1;
  mpu_config.accel_filter_level = 6;
  mpu_config.gyro_temp_filter_level = 6;
  ESP_ERROR_CHECK(mpu9250_begin(&mpu, mpu_config, 0x68, bus_handle));

  float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;
  ESP_LOGI(TAG, "Calibrating gyro (keep sensor still)...");
  for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
    while (mpu9250_update(&mpu) != 1) {
      vTaskDelay(1);
    }
    gyro_bias_x += (int16_t)mpu.gyro.x;
    gyro_bias_y += (int16_t)mpu.gyro.y;
    gyro_bias_z += (int16_t)mpu.gyro.z;
  }
  gyro_bias_x /= GYRO_CALIBRATION_SAMPLES;
  gyro_bias_y /= GYRO_CALIBRATION_SAMPLES;
  gyro_bias_z /= GYRO_CALIBRATION_SAMPLES;
  ESP_LOGI(TAG, "Gyro bias => X: %.1f, Y: %.1f, Z: %.1f", gyro_bias_x,
           gyro_bias_y, gyro_bias_z);

  espp::MadgwickFilter madgwick(0.04f);

  int64_t last_update_time = esp_timer_get_time();
  const int64_t xFrequency = 10000;
  const int64_t xLogInterval = 100000;
  int64_t logCounter = 0;

  float cal_offset_x = MAG_OFFSET_X, cal_offset_y = MAG_OFFSET_Y, cal_offset_z = MAG_OFFSET_Z;
  float cal_scale_x = MAG_SCALE_X, cal_scale_y = MAG_SCALE_Y, cal_scale_z = MAG_SCALE_Z;

  if (MAG_CALIBRATE) {
    for (int i = 5; i >= 1; i--) {
      ESP_LOGI(TAG, "MAG CALIBRATION: Get ready! Starting in %d...", i);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    float mag_min_x = 0, mag_min_y = 0, mag_min_z = 0;
    float mag_max_x = 0, mag_max_y = 0, mag_max_z = 0;
    bool first_mag = true;
    const int64_t mag_cal_duration = 15000000;
    int64_t mag_cal_start = esp_timer_get_time();

    ESP_LOGI(TAG, "MAG CALIBRATION: Rotate sensor in ALL directions NOW!");

    while (esp_timer_get_time() - mag_cal_start < mag_cal_duration) {
      if (mpu9250_update(&mpu) == 1 && mpu.config.mag_enabled) {
        float mx = (int16_t)mpu.mag.x * MAG_BASE_SENSITIVITY * mpu._mag_sensitivity[0];
        float my = (int16_t)mpu.mag.y * MAG_BASE_SENSITIVITY * mpu._mag_sensitivity[1];
        float mz = (int16_t)mpu.mag.z * MAG_BASE_SENSITIVITY * mpu._mag_sensitivity[2];
        if (first_mag) {
          mag_min_x = mag_max_x = mx;
          mag_min_y = mag_max_y = my;
          mag_min_z = mag_max_z = mz;
          first_mag = false;
        } else {
          if (mx < mag_min_x) mag_min_x = mx;
          if (mx > mag_max_x) mag_max_x = mx;
          if (my < mag_min_y) mag_min_y = my;
          if (my > mag_max_y) mag_max_y = my;
          if (mz < mag_min_z) mag_min_z = mz;
          if (mz > mag_max_z) mag_max_z = mz;
        }
      }
      vTaskDelay(1);
    }

    cal_offset_x = (mag_min_x + mag_max_x) / 2.0f;
    cal_offset_y = (mag_min_y + mag_max_y) / 2.0f;
    cal_offset_z = (mag_min_z + mag_max_z) / 2.0f;
    float avg_range = ((mag_max_x - mag_min_x) + (mag_max_y - mag_min_y) + (mag_max_z - mag_min_z)) / 3.0f;
    if (avg_range > 0.001f) {
      cal_scale_x = avg_range / (mag_max_x - mag_min_x);
      cal_scale_y = avg_range / (mag_max_y - mag_min_y);
      cal_scale_z = avg_range / (mag_max_z - mag_min_z);
    }

    ESP_LOGI(TAG, "MAG CALIBRATION RESULTS:");
    ESP_LOGI(TAG, "  Hard-iron offsets => X: %.2f Y: %.2f Z: %.2f uT", cal_offset_x, cal_offset_y, cal_offset_z);
    ESP_LOGI(TAG, "  Soft-iron scales  => X: %.3f Y: %.3f Z: %.3f", cal_scale_x, cal_scale_y, cal_scale_z);
    ESP_LOGI(TAG, "  Ranges            => X: %.2f Y: %.2f Z: %.2f uT", mag_max_x - mag_min_x, mag_max_y - mag_min_y, mag_max_z - mag_min_z);
    ESP_LOGI(TAG, "  Update main.cpp:");
    ESP_LOGI(TAG, "    MAG_OFFSET_X = %.2f", cal_offset_x);
    ESP_LOGI(TAG, "    MAG_OFFSET_Y = %.2f", cal_offset_y);
    ESP_LOGI(TAG, "    MAG_OFFSET_Z = %.2f", cal_offset_z);
    ESP_LOGI(TAG, "    MAG_SCALE_X = %.3f", cal_scale_x);
    ESP_LOGI(TAG, "    MAG_SCALE_Y = %.3f", cal_scale_y);
    ESP_LOGI(TAG, "    MAG_SCALE_Z = %.3f", cal_scale_z);
    ESP_LOGI(TAG, "Calibration complete. Halting. Update main.cpp and reflash.");
    for (;;) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
  } else {
    ESP_LOGI(TAG, "MAG CALIBRATION: Skipped (using hardcoded offsets/scales)");
  }

  for (int i = 3; i >= 1; i--) {
    ESP_LOGI(TAG, "Initializing orientation. Keep sensor still... %d", i);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  if (mpu9250_update(&mpu) == 1) {
    float init_ax = (int16_t)mpu.accel.x / ACCEL_SENSITIVITY;
    float init_ay = (int16_t)mpu.accel.y / ACCEL_SENSITIVITY;
    float init_az = (int16_t)mpu.accel.z / ACCEL_SENSITIVITY;
    float init_mx = 0, init_my = 0, init_mz = 0;
    if (mpu.config.mag_enabled) {
      init_mx = (int16_t)mpu.mag.x * MAG_BASE_SENSITIVITY * mpu._mag_sensitivity[0] - cal_offset_x;
      init_my = (int16_t)mpu.mag.y * MAG_BASE_SENSITIVITY * mpu._mag_sensitivity[1] - cal_offset_y;
      init_mz = (int16_t)mpu.mag.z * MAG_BASE_SENSITIVITY * mpu._mag_sensitivity[2] - cal_offset_z;
      init_mx *= cal_scale_x;
      init_my *= cal_scale_y;
      init_mz *= cal_scale_z;
    }
    for (int i = 0; i < 200; i++) {
      madgwick.update(0.01f, init_ax, init_ay, init_az, 0, 0, 0, init_mx, init_my, init_mz);
    }
    float init_pitch, init_roll, init_yaw;
    madgwick.get_euler(init_pitch, init_roll, init_yaw);
    ESP_LOGI(TAG, "Initial orientation => Pitch: %.2f° Roll: %.2f° Yaw: %.2f°", init_pitch, init_roll, init_yaw);
  }

  for (;;) {
    int64_t now = esp_timer_get_time();
    if (now - last_update_time >= xFrequency) {
      if (mpu9250_update(&mpu) == 1) {
        float ax = (int16_t)mpu.accel.x / ACCEL_SENSITIVITY;
        float ay = (int16_t)mpu.accel.y / ACCEL_SENSITIVITY;
        float az = (int16_t)mpu.accel.z / ACCEL_SENSITIVITY;

        float gx = (((int16_t)mpu.gyro.x - gyro_bias_x) / GYRO_SENSITIVITY) * DEG_TO_RAD;
        float gy = (((int16_t)mpu.gyro.y - gyro_bias_y) / GYRO_SENSITIVITY) * DEG_TO_RAD;
        float gz = (((int16_t)mpu.gyro.z - gyro_bias_z) / GYRO_SENSITIVITY) * DEG_TO_RAD;

        float mx = 0, my = 0, mz = 0;
        if (mpu.config.mag_enabled) {
          mx = (int16_t)mpu.mag.x * MAG_BASE_SENSITIVITY * mpu._mag_sensitivity[0] - cal_offset_x;
          my = (int16_t)mpu.mag.y * MAG_BASE_SENSITIVITY * mpu._mag_sensitivity[1] - cal_offset_y;
          mz = (int16_t)mpu.mag.z * MAG_BASE_SENSITIVITY * mpu._mag_sensitivity[2] - cal_offset_z;
          mx *= cal_scale_x;
          my *= cal_scale_y;
          mz *= cal_scale_z;
        }

        float dt = (now - last_update_time) / 1000000.0f;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.5f) dt = 0.5f;
        madgwick.update(dt, ax, ay, az, gx, gy, gz, mx, my, mz);

        logCounter += xFrequency;
        if (logCounter >= xLogInterval) {
          logCounter = 0;
          int16_t raw_ax = (int16_t)mpu.accel.x;
          int16_t raw_ay = (int16_t)mpu.accel.y;
          int16_t raw_az = (int16_t)mpu.accel.z;
          int16_t raw_mx = (int16_t)mpu.mag.x;
          int16_t raw_my = (int16_t)mpu.mag.y;
          int16_t raw_mz = (int16_t)mpu.mag.z;
          ESP_LOGI(TAG, "Raw Accel: X=%d Y=%d Z=%d | Norm: X=%.3f Y=%.3f Z=%.3f",
                   raw_ax, raw_ay, raw_az, ax, ay, az);
          ESP_LOGI(TAG, "Raw Mag:   X=%d Y=%d Z=%d | uT: X=%.2f Y=%.2f Z=%.2f",
                   raw_mx, raw_my, raw_mz, mx, my, mz);
          float pitch, roll, yaw;
          madgwick.get_euler(pitch, roll, yaw);
          ESP_LOGI(TAG, "Pitch: %6.2f°  Roll: %6.2f°  Yaw: %6.2f°", pitch, roll,
                   yaw);
        }
        last_update_time = now;
      }
    }
    vTaskDelay(1);
  }
}

extern "C" void app_main(void) { xTaskCreate(vImuTask, "IMU", 4096, NULL, 1, NULL); }
