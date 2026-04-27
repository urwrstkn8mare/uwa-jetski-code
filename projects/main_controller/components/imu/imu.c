#include "imu.h"

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

#define SCL_PIN 33
#define SDA_PIN 32
#define MPU9250_ADDR 0x68

#define PRE_CONVERGE_US 8000000
#define BIAS_SAMPLES 200
#define LOG_INTERVAL_US 100000

static const char *TAG = "imu";

static SemaphoreHandle_t s_mutex;
static float s_pitch = 0.0f;
static float s_roll = 0.0f;
static bool s_initialized = false;

/* Zero-reference quaternion (float, normalized) */
static float s_qz[4] = {1.0f, 0.0f, 0.0f, 0.0f};

/* DMP outputs quaternions in Q30 format */
static inline float q30_to_float(long q) { return q / 1073741824.0f; }

static inline void quat_to_pitch_roll(float w, float x, float y, float z,
                                      float *pitch, float *roll) {
    *pitch = asinf(-2.0f * (x * z - w * y)) * 180.0f / (float)M_PI;
    *roll = atan2f(2.0f * (w * x + y * z),
                   w * w - x * x - y * y + z * z) *
            180.0f / (float)M_PI;
}

/* Multiply two quaternions: result = a * b */
static inline void quat_mul(const float a[4], const float b[4], float out[4]) {
    float aw = a[0], ax = a[1], ay = a[2], az = a[3];
    float bw = b[0], bx = b[1], by = b[2], bz = b[3];
    out[0] = aw * bw - ax * bx - ay * by - az * bz;
    out[1] = aw * bx + ax * bw + ay * bz - az * by;
    out[2] = aw * by - ax * bz + ay * bw + az * bx;
    out[3] = aw * bz + ax * by - ay * bx + az * bw;
}

/* Inverse of a unit quaternion (conjugate) */
static inline void quat_inv(const float q[4], float out[4]) {
    out[0] = q[0];
    out[1] = -q[1];
    out[2] = -q[2];
    out[3] = -q[3];
}

/* Normalize a quaternion */
static inline void quat_normalize(float q[4]) {
    float norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (norm > 0.0f) {
        q[0] /= norm;
        q[1] /= norm;
        q[2] /= norm;
        q[3] /= norm;
    }
}

/* Compute relative quaternion: out = q_ref_inv * q_current */
static inline void quat_relative(const float q_ref[4], const float q_cur[4],
                                 float out[4]) {
    float q_ref_inv[4];
    quat_inv(q_ref, q_ref_inv);
    quat_mul(q_ref_inv, q_cur, out);
    quat_normalize(out);
}

static void imu_task(void *pvParameters) {
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = SCL_PIN,
        .sda_io_num = SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags = {.enable_internal_pullup = true}};
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    esp32_i2c_init(bus_handle, MPU9250_ADDR);

    ESP_LOGI(TAG, "Initializing MPU9250 DMP...");
    struct int_param_s int_param = {0};
    if (mpu_init(&int_param) != 0) {
        ESP_LOGE(TAG, "mpu_init failed");
        vTaskDelete(NULL);
    }

    if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0) {
        ESP_LOGE(TAG, "mpu_set_sensors failed");
        vTaskDelete(NULL);
    }

    if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0) {
        ESP_LOGE(TAG, "mpu_configure_fifo failed");
        vTaskDelete(NULL);
    }

    if (mpu_set_sample_rate(200) != 0) {
        ESP_LOGE(TAG, "mpu_set_sample_rate failed");
        vTaskDelete(NULL);
    }

    /* Gyro bias calibration */
    ESP_LOGI(TAG, "Calibrating gyro bias (keep sensor still)...");
    long gyro_bias[3] = {0, 0, 0};
    for (int i = 0; i < BIAS_SAMPLES; i++) {
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
    gyro_bias[0] /= BIAS_SAMPLES;
    gyro_bias[1] /= BIAS_SAMPLES;
    gyro_bias[2] /= BIAS_SAMPLES;
    ESP_LOGI(TAG, "Gyro bias => X: %ld, Y: %ld, Z: %ld", gyro_bias[0],
             gyro_bias[1], gyro_bias[2]);

    if (dmp_set_gyro_bias(gyro_bias) != 0) {
        ESP_LOGW(TAG, "dmp_set_gyro_bias failed, continuing without bias correction");
    } else {
        ESP_LOGI(TAG, "Gyro bias loaded into DMP");
    }

    if (dmp_load_motion_driver_firmware() != 0) {
        ESP_LOGE(TAG, "dmp_load_motion_driver_firmware failed");
        vTaskDelete(NULL);
    }

    if (dmp_set_fifo_rate(200) != 0) {
        ESP_LOGE(TAG, "dmp_set_fifo_rate failed");
        vTaskDelete(NULL);
    }

    unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL;
    if (dmp_enable_feature(dmp_features) != 0) {
        ESP_LOGE(TAG, "dmp_enable_feature failed");
        vTaskDelete(NULL);
    }

    if (mpu_set_dmp_state(1) != 0) {
        ESP_LOGE(TAG, "mpu_set_dmp_state failed");
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "DMP initialized. Pre-converging (keep still for %.1fs)...",
             PRE_CONVERGE_US / 1000000.0f);
    int64_t converge_start = esp_timer_get_time();
    while (esp_timer_get_time() - converge_start < PRE_CONVERGE_US) {
        short g[3], a[3];
        long q[4];
        unsigned long ts;
        short sens;
        unsigned char more_data;
        dmp_read_fifo(g, a, q, &ts, &sens, &more_data);
        vTaskDelay(1);
    }
    ESP_LOGI(TAG, "Pre-convergence complete. Zeroing sensor...");

    /* Capture zero-reference quaternion */
    short g[3], a[3];
    long q[4];
    unsigned long ts;
    short sens;
    unsigned char more;
    if (dmp_read_fifo(g, a, q, &ts, &sens, &more) == 0 &&
        (sens & INV_WXYZ_QUAT)) {
        s_qz[0] = q30_to_float(q[0]);
        s_qz[1] = q30_to_float(q[1]);
        s_qz[2] = q30_to_float(q[2]);
        s_qz[3] = q30_to_float(q[3]);
        quat_normalize(s_qz);
    } else {
        ESP_LOGW(TAG, "Failed to read quaternion for zeroing, using identity");
        s_qz[0] = 1.0f; s_qz[1] = 0.0f; s_qz[2] = 0.0f; s_qz[3] = 0.0f;
    }
    ESP_LOGI(TAG, "Sensor zeroed. Ready.");
    s_initialized = true;

    int64_t last_log = esp_timer_get_time();
    for (;;) {
        if (dmp_read_fifo(g, a, q, &ts, &sens, &more) == 0) {
            if (sens & INV_WXYZ_QUAT) {
                float q_cur[4] = {
                    q30_to_float(q[0]),
                    q30_to_float(q[1]),
                    q30_to_float(q[2]),
                    q30_to_float(q[3]),
                };
                quat_normalize(q_cur);

                float q_rel[4];
                quat_relative(s_qz, q_cur, q_rel);

                float pitch, roll;
                quat_to_pitch_roll(q_rel[0], q_rel[1], q_rel[2], q_rel[3],
                                   &pitch, &roll);

                if (xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
                    s_pitch = pitch;
                    s_roll = roll;
                    xSemaphoreGive(s_mutex);
                }

                int64_t now = esp_timer_get_time();
                if (now - last_log >= LOG_INTERVAL_US) {
                    ESP_LOGI(TAG, "Pitch: %6.2f deg  Roll: %6.2f deg", pitch,
                             roll);
                    last_log = now;
                }
            }
        }
        vTaskDelay(1);
    }
}

esp_err_t imu_init(void) {
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }

    xTaskCreate(imu_task, "imu_task", 8192, NULL, 1, NULL);

    /* Wait until the IMU task signals it is ready */
    while (!s_initialized) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    return ESP_OK;
}

esp_err_t imu_get_pitch_roll(float *pitch, float *roll) {
    if (!s_initialized) {
        return ESP_FAIL;
    }
    if (pitch == NULL || roll == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }
    *pitch = s_pitch;
    *roll = s_roll;
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}
