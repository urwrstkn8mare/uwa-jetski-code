/*
 * ICM-20948 attitude uses the on-chip DMP only (no raw accel/gyro integration in
 * this file). Flow: DMP sensor ORIENTATION -> Quat9 in FIFO -> inv_icm20948_read_dmp_data.
 */
#include "imu.h"

#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "icm20948.h"
#include "icm20948_dmp.h"
#include "icm20948_i2c.h"
#include <math.h>
#include <string.h>

static const char *TAG = "imu";

#define PRE_CONVERGE_US 8000000
#define LOG_INTERVAL_US 100000

#define ICM_MAX_CHECK_ID_ROUNDS 24  /* x 500ms ≈ 12s */
#define ICM_MAX_WHOAMI_ROUNDS 40    /* x 200ms ≈ 8s */

static SemaphoreHandle_t s_mutex;
static float s_pitch;
static float s_roll;
static float s_yaw;
static bool s_ready;
static bool s_failed;
/** Worker task is running until it exits via failure (successful path loops forever). */
static volatile bool s_imu_task_alive;

static float s_qz[4] = {1.0f, 0.0f, 0.0f, 0.0f};

static icm20948_device_t s_icm;
static i2c_config_t s_i2c_cfg;
static icm0948_config_i2c_t s_icm_i2c = {
    .i2c_port = I2C_NUM_0,
    .i2c_addr = ICM_20948_I2C_ADDR_AD0,
};

static void quat_to_pitch_roll_yaw(float w, float x, float y, float z, float *pitch, float *roll, float *yaw) {
  *pitch = asinf(-2.0f * (x * z - w * y)) * 180.0f / (float)M_PI;
  *roll = atan2f(2.0f * (w * x + y * z), w * w - x * x - y * y + z * z) * 180.0f / (float)M_PI;
  *yaw = atan2f(2.0f * (w * z + x * y), w * w + x * x - y * y - z * z) * 180.0f / (float)M_PI;
}

static void quat_mul(const float a[4], const float b[4], float out[4]) {
  float aw = a[0], ax = a[1], ay = a[2], az = a[3];
  float bw = b[0], bx = b[1], by = b[2], bz = b[3];
  out[0] = aw * bw - ax * bx - ay * by - az * bz;
  out[1] = aw * bx + ax * bw + ay * bz - az * by;
  out[2] = aw * by - ax * bz + ay * bw + az * bx;
  out[3] = aw * bz + ax * by - ay * bx + az * bw;
}

static void quat_inv(const float q[4], float out[4]) {
  out[0] = q[0];
  out[1] = -q[1];
  out[2] = -q[2];
  out[3] = -q[3];
}

static void quat_normalize(float q[4]) {
  float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n > 0.0f) {
    q[0] /= n;
    q[1] /= n;
    q[2] /= n;
    q[3] /= n;
  }
}

static void quat_relative(const float q_ref[4], const float q_cur[4], float out[4]) {
  float inv[4];
  quat_inv(q_ref, inv);
  quat_mul(inv, q_cur, out);
  quat_normalize(out);
}

static bool init_dmp(icm20948_device_t *icm) {
  bool ok = true;
  ok &= (icm20948_init_dmp_sensor_with_defaults(icm) == ICM_20948_STAT_OK);
  ok &= (inv_icm20948_enable_dmp_sensor(icm, INV_ICM20948_SENSOR_ORIENTATION, 1) == ICM_20948_STAT_OK);
  ok &= (inv_icm20948_set_dmp_sensor_period(icm, DMP_ODR_Reg_Quat9, 0) == ICM_20948_STAT_OK);
  ok &= (icm20948_enable_fifo(icm, true) == ICM_20948_STAT_OK);
  ok &= (icm20948_enable_dmp(icm, 1) == ICM_20948_STAT_OK);
  ok &= (icm20948_reset_dmp(icm) == ICM_20948_STAT_OK);
  ok &= (icm20948_reset_fifo(icm) == ICM_20948_STAT_OK);
  if (!ok) {
    ESP_LOGE(TAG, "DMP configuration failed");
    return false;
  }
  ESP_LOGI(TAG, "DMP enabled (Quat9 orientation)");
  return true;
}

static void imu_task(void *arg) {
  (void)arg;

  s_imu_task_alive = true;

  s_icm_i2c.i2c_port = (i2c_port_t)CONFIG_ICM_I2C_PORT_NUM;
#ifdef CONFIG_ICM_I2C_ADDR_AD1
  s_icm_i2c.i2c_addr = ICM_20948_I2C_ADDR_AD1;
#else
  s_icm_i2c.i2c_addr = ICM_20948_I2C_ADDR_AD0;
#endif

  s_i2c_cfg.mode = I2C_MODE_MASTER;
  s_i2c_cfg.sda_io_num = (gpio_num_t)CONFIG_ICM_I2C_SDA_GPIO;
  s_i2c_cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
  s_i2c_cfg.scl_io_num = (gpio_num_t)CONFIG_ICM_I2C_SCL_GPIO;
  s_i2c_cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
  s_i2c_cfg.master.clk_speed = 400000;
  s_i2c_cfg.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

  esp_err_t er = i2c_param_config(s_icm_i2c.i2c_port, &s_i2c_cfg);
  if (er != ESP_OK) {
    ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(er));
    s_imu_task_alive = false;
    s_failed = true;
    vTaskDelete(NULL);
    return;
  }
  er = i2c_driver_install(s_icm_i2c.i2c_port, s_i2c_cfg.mode, 0, 0, 0);
  if (er != ESP_OK) {
    ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(er));
    s_imu_task_alive = false;
    s_failed = true;
    vTaskDelete(NULL);
    return;
  }

  icm20948_init_i2c(&s_icm, &s_icm_i2c);

  int id_pass = 0;
  while (icm20948_check_id(&s_icm) != ICM_20948_STAT_OK) {
    if (++id_pass >= ICM_MAX_CHECK_ID_ROUNDS) {
      ESP_LOGW(TAG, "ICM-20948 not responding on I2C — IMU path disabled");
      (void)i2c_driver_delete(s_icm_i2c.i2c_port);
      s_imu_task_alive = false;
      s_failed = true;
      vTaskDelete(NULL);
      return;
    }
    ESP_LOGW(TAG, "check id failed, retry (%d/%d)", id_pass, ICM_MAX_CHECK_ID_ROUNDS);
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  uint8_t whoami = 0;
  int who_pass = 0;
  while (icm20948_get_who_am_i(&s_icm, &whoami) != ICM_20948_STAT_OK || whoami != ICM_20948_WHOAMI) {
    if (++who_pass >= ICM_MAX_WHOAMI_ROUNDS) {
      ESP_LOGW(TAG, "ICM-20948 WHOAMI bad (0x%02x) — IMU path disabled", whoami);
      (void)i2c_driver_delete(s_icm_i2c.i2c_port);
      s_imu_task_alive = false;
      s_failed = true;
      vTaskDelete(NULL);
      return;
    }
    ESP_LOGW(TAG, "WHOAMI mismatch (0x%02x) retry", whoami);
    vTaskDelay(pdMS_TO_TICKS(200));
  }

  icm20948_sw_reset(&s_icm);
  vTaskDelay(pdMS_TO_TICKS(250));

  icm20948_internal_sensor_id_bm sensors =
      (icm20948_internal_sensor_id_bm)(ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR);
  icm20948_set_sample_mode(&s_icm, sensors, SAMPLE_MODE_CONTINUOUS);

  icm20948_fss_t fss = {.a = GPM_2, .g = DPS_250};
  icm20948_set_full_scale(&s_icm, sensors, fss);

  icm20948_dlpcfg_t dlp = {.a = ACC_D473BW_N499BW, .g = GYR_D361BW4_N376BW5};
  icm20948_set_dlpf_cfg(&s_icm, sensors, dlp);
  icm20948_enable_dlpf(&s_icm, ICM_20948_INTERNAL_ACC, false);
  icm20948_enable_dlpf(&s_icm, ICM_20948_INTERNAL_GYR, false);
  icm20948_sleep(&s_icm, false);
  icm20948_low_power(&s_icm, false);

  if (!init_dmp(&s_icm)) {
    ESP_LOGW(TAG, "DMP setup failed — IMU path disabled");
    (void)i2c_driver_delete(s_icm_i2c.i2c_port);
    s_imu_task_alive = false;
    s_failed = true;
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGI(TAG, "Pre-converging DMP (keep still %.1fs)...", PRE_CONVERGE_US / 1e6f);
  int64_t t0 = esp_timer_get_time();
  while (esp_timer_get_time() - t0 < (int64_t)PRE_CONVERGE_US) {
    icm_20948_DMP_data_t d;
    (void)inv_icm20948_read_dmp_data(&s_icm, &d);
    vTaskDelay(1);
  }

  icm_20948_DMP_data_t zdata;
  icm20948_status_e st = inv_icm20948_read_dmp_data(&s_icm, &zdata);
  if (st == ICM_20948_STAT_OK || st == ICM_20948_STAT_FIFO_MORE_DATA_AVAIL) {
    if ((zdata.header & DMP_header_bitmap_Quat9) != 0) {
      double q1 = (double)zdata.Quat9.Data.Q1 / 1073741824.0;
      double q2 = (double)zdata.Quat9.Data.Q2 / 1073741824.0;
      double q3 = (double)zdata.Quat9.Data.Q3 / 1073741824.0;
      double q0sq = 1.0 - (q1 * q1 + q2 * q2 + q3 * q3);
      double q0 = (q0sq > 0.0) ? sqrt(q0sq) : 0.0;
      s_qz[0] = (float)q0;
      s_qz[1] = (float)q1;
      s_qz[2] = (float)q2;
      s_qz[3] = (float)q3;
      quat_normalize(s_qz);
      ESP_LOGI(TAG, "Zero quaternion captured");
    } else {
      ESP_LOGW(TAG, "No Quat9 in first frame; using identity zero");
    }
  }

  s_ready = true;
  int64_t last_log = esp_timer_get_time();

  for (;;) {
    icm_20948_DMP_data_t data;
    icm20948_status_e status = inv_icm20948_read_dmp_data(&s_icm, &data);
    if (status == ICM_20948_STAT_OK || status == ICM_20948_STAT_FIFO_MORE_DATA_AVAIL) {
      if ((data.header & DMP_header_bitmap_Quat9) != 0) {
        double q1 = (double)data.Quat9.Data.Q1 / 1073741824.0;
        double q2 = (double)data.Quat9.Data.Q2 / 1073741824.0;
        double q3 = (double)data.Quat9.Data.Q3 / 1073741824.0;
        double q0sq = 1.0 - (q1 * q1 + q2 * q2 + q3 * q3);
        double q0d = (q0sq > 0.0) ? sqrt(q0sq) : 0.0;
        float q_cur[4] = {(float)q0d, (float)q1, (float)q2, (float)q3};
        quat_normalize(q_cur);

        float q_rel[4];
        quat_relative(s_qz, q_cur, q_rel);

        float pitch, roll, yaw;
        quat_to_pitch_roll_yaw(q_rel[0], q_rel[1], q_rel[2], q_rel[3], &pitch, &roll, &yaw);

        if (xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
          s_pitch = pitch;
          s_roll = roll;
          s_yaw = yaw;
          xSemaphoreGive(s_mutex);
        }

        int64_t now = esp_timer_get_time();
        if (now - last_log >= LOG_INTERVAL_US) {
          last_log = now;
        }
      }
    }
    if (status != ICM_20948_STAT_FIFO_MORE_DATA_AVAIL) {
      vTaskDelay(pdMS_TO_TICKS(2));
    }
  }
}

esp_err_t imu_init(void) {
#if CONFIG_IMU_SKIP_HW
  ESP_LOGW(TAG, "IMU disabled by Kconfig (CONFIG_IMU_SKIP_HW)");
  return ESP_FAIL;
#endif

  static bool mutex_ready;
  if (!mutex_ready) {
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
      return ESP_ERR_NO_MEM;
    }
    mutex_ready = true;
  }

  /* Spawn once per logical attempt — if a worker is already alive, only wait longer. */
  if (!s_imu_task_alive) {
    s_ready = false;
    s_failed = false;
    BaseType_t r = xTaskCreate(imu_task, "imu_icm20948", 12288, NULL, 5, NULL);
    if (r != pdPASS) {
      s_failed = true;
      ESP_LOGW(TAG, "imu task create failed");
      return ESP_ERR_NO_MEM;
    }
  }

  /* Worst case: ID(~12s) + WHOAMI(~8s) + converge(8s) + margin — do not underestimate */
  const int max_wait = 500;
  int waited = 0;
  while (!s_ready && !s_failed && waited < max_wait) {
    vTaskDelay(pdMS_TO_TICKS(100));
    waited++;
  }
  if (s_failed) {
    ESP_LOGW(TAG, "IMU init failed (sensor missing or bus error)");
    return ESP_FAIL;
  }
  if (!s_ready) {
    ESP_LOGW(TAG, "IMU init timed out (still converging / slow bus)");
    return ESP_ERR_TIMEOUT;
  }
  return ESP_OK;
}

esp_err_t imu_get_pitch_roll_yaw(float *pitch, float *roll, float *yaw) {
  if (!s_ready) {
    return ESP_ERR_INVALID_STATE;
  }
  if (pitch == NULL || roll == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
    return ESP_FAIL;
  }
  *pitch = s_pitch;
  *roll = s_roll;
  if (yaw != NULL) {
    *yaw = s_yaw;
  }
  xSemaphoreGive(s_mutex);
  return ESP_OK;
}

esp_err_t imu_get_pitch_roll(float *pitch, float *roll) {
  return imu_get_pitch_roll_yaw(pitch, roll, NULL);
}
