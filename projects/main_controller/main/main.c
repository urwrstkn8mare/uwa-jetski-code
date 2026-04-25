#include "FreeRTOSConfig.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "esp32_mpu9250_i2c.h"

#define SCL_PIN 39
#define SDA_PIN 38
#define MPU9250_ADDR 0x68

static const char *TAG = "mpu_dmp";

static inline float quaternion_to_pitch(long q0, long q1, long q2, long q3) {
  // DMP quaternions are in Q30 format (2^30 = 1073741824)
  float w = q0 / 1073741824.0f;
  float x = q1 / 1073741824.0f;
  float y = q2 / 1073741824.0f;
  float z = q3 / 1073741824.0f;
  return atan2f(2.0f * (w * x + y * z), w * w - x * x - y * y + z * z) * 180.0f / (float)M_PI;
}

static inline float quaternion_to_roll(long q0, long q1, long q2, long q3) {
  float w = q0 / 1073741824.0f;
  float x = q1 / 1073741824.0f;
  float y = q2 / 1073741824.0f;
  float z = q3 / 1073741824.0f;
  return asinf(-2.0f * (x * z - w * y)) * 180.0f / (float)M_PI;
}

static void vImuTask(void *pvParameters) {
  i2c_master_bus_config_t bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = SCL_PIN,
    .sda_io_num = SDA_PIN,
    .glitch_ignore_cnt = 7,
    .flags = { .enable_internal_pullup = true }
  };
  i2c_master_bus_handle_t bus_handle;
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

  esp32_i2c_init(bus_handle, MPU9250_ADDR);

  ESP_LOGI(TAG, "Initializing MPU9250 DMP...");
  struct int_param_s int_param = {0};
  if (mpu_init(&int_param) != 0) {
    ESP_LOGE(TAG, "mpu_init failed");
    return;
  }

  if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0) {
    ESP_LOGE(TAG, "mpu_set_sensors failed");
    return;
  }

  if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0) {
    ESP_LOGE(TAG, "mpu_configure_fifo failed");
    return;
  }

  if (mpu_set_sample_rate(200) != 0) {
    ESP_LOGE(TAG, "mpu_set_sample_rate failed");
    return;
  }

  // Calibrate gyro bias before enabling DMP
  ESP_LOGI(TAG, "Calibrating gyro bias (keep sensor still)...");
  long gyro_bias[3] = {0, 0, 0};
  const int bias_samples = 200;
  for (int i = 0; i < bias_samples; i++) {
    short data[3];
    unsigned long ts;
    while (mpu_get_gyro_reg(data, &ts) != 0) {
      vTaskDelay(1);
    }
    gyro_bias[0] += data[0];
    gyro_bias[1] += data[1];
    gyro_bias[2] += data[2];
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
  gyro_bias[0] /= bias_samples;
  gyro_bias[1] /= bias_samples;
  gyro_bias[2] /= bias_samples;
  ESP_LOGI(TAG, "Gyro bias => X: %ld, Y: %ld, Z: %ld", gyro_bias[0], gyro_bias[1], gyro_bias[2]);

  if (dmp_set_gyro_bias(gyro_bias) != 0) {
    ESP_LOGW(TAG, "dmp_set_gyro_bias failed, continuing without bias correction");
  } else {
    ESP_LOGI(TAG, "Gyro bias loaded into DMP");
  }

  ESP_LOGI(TAG, "Loading DMP firmware...");
  if (dmp_load_motion_driver_firmware() != 0) {
    ESP_LOGE(TAG, "dmp_load_motion_driver_firmware failed");
    return;
  }

  if (dmp_set_fifo_rate(200) != 0) {
    ESP_LOGE(TAG, "dmp_set_fifo_rate failed");
    return;
  }

  unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL;
  if (dmp_enable_feature(dmp_features) != 0) {
    ESP_LOGE(TAG, "dmp_enable_feature failed");
    return;
  }

  if (mpu_set_dmp_state(1) != 0) {
    ESP_LOGE(TAG, "mpu_set_dmp_state failed");
    return;
  }

  ESP_LOGI(TAG, "DMP initialized successfully!");

  ESP_LOGI(TAG, "Pre-converging DMP (keep sensor still for 8s)...");
  int64_t converge_start = esp_timer_get_time();
  const int64_t converge_duration = 8000000;
  while (esp_timer_get_time() - converge_start < converge_duration) {
    short g[3], a[3];
    long q[4];
    unsigned long ts;
    short sens;
    unsigned char more_data;
    dmp_read_fifo(g, a, q, &ts, &sens, &more_data);
    vTaskDelay(1);
  }
  ESP_LOGI(TAG, "DMP pre-convergence complete!");

  short gyro[3], accel[3];
  long quat[4];
  unsigned long timestamp;
  short sensors;
  unsigned char more;
  int64_t last_log = esp_timer_get_time();

  for (;;) {
    if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more) == 0) {
      if (sensors & INV_WXYZ_QUAT) {
        float pitch = quaternion_to_pitch(quat[0], quat[1], quat[2], quat[3]);
        float roll = quaternion_to_roll(quat[0], quat[1], quat[2], quat[3]);

        int64_t now = esp_timer_get_time();
        if (now - last_log >= 100000) {
          ESP_LOGI(TAG, "Pitch: %6.2f deg  Roll: %6.2f deg", pitch, roll);
          last_log = now;
        }
      }
    }
    vTaskDelay(1);
  }
}

void app_main(void) { xTaskCreate(vImuTask, "IMU", 8192, NULL, 1, NULL); }
