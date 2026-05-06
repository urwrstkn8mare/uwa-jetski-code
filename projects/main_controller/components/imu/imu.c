#include "imu.h"

#include <math.h>
#include <stdio.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "icm20948.h"
#include "icm20948_i2c.h"
#include "sdkconfig.h"

static const char *TAG = "imu";

#define ICM_I2C_PORT ((i2c_port_num_t)CONFIG_IMU_I2C_PORT)
#define ICM_I2C_SDA_GPIO ((gpio_num_t)CONFIG_IMU_I2C_SDA_GPIO)
#define ICM_I2C_SCL_GPIO ((gpio_num_t)CONFIG_IMU_I2C_SCL_GPIO)
#define ICM_I2C_FREQ_HZ CONFIG_IMU_I2C_FREQ_HZ
#define ICM_I2C_ADDRESS CONFIG_IMU_I2C_ADDRESS

static const double DMP_QUAT_Q30_SCALE = (double)(1ULL << 30);

static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_i2c_dev = NULL;
static icm20948_device_t s_icm_dev;
static icm0948_config_i2c_t s_icm_cfg;
static SemaphoreHandle_t s_mutex;
static bool s_imu_initialized = false;
static volatile bool s_reader_task_started = false;
static float s_pitch_deg = 0.0f;
static float s_roll_deg = 0.0f;
static float s_yaw_deg = 0.0f;

static void quaternion_to_rpy(double q0, double q1, double q2, double q3, float *pitch, float *roll, float *yaw) {
    const double sinr_cosp = 2.0 * ((q0 * q1) + (q2 * q3));
    const double cosr_cosp = 1.0 - (2.0 * ((q1 * q1) + (q2 * q2)));
    const double sinp = 2.0 * ((q0 * q2) - (q3 * q1));
    const double siny_cosp = 2.0 * ((q0 * q3) + (q1 * q2));
    const double cosy_cosp = 1.0 - (2.0 * ((q2 * q2) + (q3 * q3)));

    *roll = (float)(atan2(sinr_cosp, cosr_cosp) * (180.0 / M_PI));
    if (fabs(sinp) >= 1.0) {
        *pitch = (float)(copysign(M_PI / 2.0, sinp) * (180.0 / M_PI));
    } else {
        *pitch = (float)(asin(sinp) * (180.0 / M_PI));
    }
    *yaw = (float)(atan2(siny_cosp, cosy_cosp) * (180.0 / M_PI));
}

static esp_err_t init_icm_i2c(void) {
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = ICM_I2C_PORT,
        .sda_io_num = ICM_I2C_SDA_GPIO,
        .scl_io_num = ICM_I2C_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_i2c_bus), TAG, "failed to create i2c bus");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ICM_I2C_ADDRESS,
        .scl_speed_hz = ICM_I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_i2c_dev), TAG, "failed to add i2c device");
    return ESP_OK;
}

static esp_err_t init_icm20948_dmp(icm20948_device_t *icm_dev) {
    icm20948_status_e st = ICM_20948_STAT_OK;

    st = icm20948_init_dmp_sensor_with_defaults(icm_dev);
    ESP_RETURN_ON_FALSE(st == ICM_20948_STAT_OK, ESP_FAIL, TAG, "DMP init defaults failed: %d", st);

    st = inv_icm20948_enable_dmp_sensor(icm_dev, INV_ICM20948_SENSOR_ORIENTATION, 1);
    ESP_RETURN_ON_FALSE(st == ICM_20948_STAT_OK, ESP_FAIL, TAG, "DMP enable orientation failed: %d", st);

    st = inv_icm20948_set_dmp_sensor_period(icm_dev, DMP_ODR_Reg_Quat9, 0);
    ESP_RETURN_ON_FALSE(st == ICM_20948_STAT_OK, ESP_FAIL, TAG, "DMP set quat9 ODR failed: %d", st);

    st = icm20948_enable_fifo(icm_dev, true);
    ESP_RETURN_ON_FALSE(st == ICM_20948_STAT_OK, ESP_FAIL, TAG, "DMP enable FIFO failed: %d", st);

    st = icm20948_enable_dmp(icm_dev, true);
    ESP_RETURN_ON_FALSE(st == ICM_20948_STAT_OK, ESP_FAIL, TAG, "DMP enable failed: %d", st);

    st = icm20948_reset_dmp(icm_dev);
    ESP_RETURN_ON_FALSE(st == ICM_20948_STAT_OK, ESP_FAIL, TAG, "DMP reset failed: %d", st);

    st = icm20948_reset_fifo(icm_dev);
    ESP_RETURN_ON_FALSE(st == ICM_20948_STAT_OK, ESP_FAIL, TAG, "DMP FIFO reset failed: %d", st);

    return ESP_OK;
}

static bool rpy_is_valid(float pitch, float roll, float yaw) {
    return isfinite(pitch) && isfinite(roll) && isfinite(yaw) && fabsf(pitch) <= 180.0f && fabsf(roll) <= 180.0f &&
           fabsf(yaw) <= 180.0f;
}

static void imu_reader_task(void *arg) {
    (void)arg;
    for (;;) {
        icm_20948_DMP_data_t dmp_data;
        icm20948_status_e dmp_status = inv_icm20948_read_dmp_data(&s_icm_dev, &dmp_data);
        if (dmp_status == ICM_20948_STAT_OK || dmp_status == ICM_20948_STAT_FIFO_MORE_DATA_AVAIL) {
            if ((dmp_data.header & DMP_header_bitmap_Quat9) > 0) {
                const double q1 = ((double)dmp_data.Quat9.Data.Q1) / DMP_QUAT_Q30_SCALE;
                const double q2 = ((double)dmp_data.Quat9.Data.Q2) / DMP_QUAT_Q30_SCALE;
                const double q3 = ((double)dmp_data.Quat9.Data.Q3) / DMP_QUAT_Q30_SCALE;
                double q0_sq = 1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3));
                if (q0_sq < 0.0) {
                    q0_sq = 0.0;
                }

                float pitch = 0.0f;
                float roll = 0.0f;
                float yaw = 0.0f;
                quaternion_to_rpy(sqrt(q0_sq), q1, q2, q3, &pitch, &roll, &yaw);

                if (rpy_is_valid(pitch, roll, yaw) && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    s_pitch_deg = pitch;
                    s_roll_deg = roll;
                    s_yaw_deg = yaw;
                    xSemaphoreGive(s_mutex);
                }
            }
        }

        if (dmp_status != ICM_20948_STAT_FIFO_MORE_DATA_AVAIL) {
            vTaskDelay(pdMS_TO_TICKS(5));
        } else {
            taskYIELD();
        }
    }
}

esp_err_t imu_init(void) {
#if CONFIG_IMU_SKIP_HW
    ESP_LOGW(TAG, "IMU disabled by Kconfig (CONFIG_IMU_SKIP_HW)");
    return ESP_FAIL;
#endif

    if (s_imu_initialized) {
        return ESP_OK;
    }

    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(s_mutex != NULL, ESP_ERR_NO_MEM, TAG, "failed to create mutex");
    }

    ESP_RETURN_ON_ERROR(init_icm_i2c(), TAG, "failed to init i2c");

    s_icm_cfg = (icm0948_config_i2c_t){
        .i2c_port = ICM_I2C_PORT,
        .i2c_addr = ICM_I2C_ADDRESS,
        .bus_handle = s_i2c_bus,
        .dev_handle = s_i2c_dev,
    };
    icm20948_init_i2c(&s_icm_dev, &s_icm_cfg);

    uint8_t who_am_i = 0x00;
    ESP_RETURN_ON_FALSE(icm20948_get_who_am_i(&s_icm_dev, &who_am_i) == ICM_20948_STAT_OK, ESP_FAIL, TAG, "WHOAMI read failed");
    ESP_RETURN_ON_FALSE(who_am_i == ICM_20948_WHOAMI, ESP_FAIL, TAG, "WHOAMI mismatch: 0x%02X", who_am_i);

    ESP_RETURN_ON_FALSE(icm20948_sw_reset(&s_icm_dev) == ICM_20948_STAT_OK, ESP_FAIL, TAG, "sensor reset failed");
    vTaskDelay(pdMS_TO_TICKS(250));

    ESP_RETURN_ON_FALSE(icm20948_sleep(&s_icm_dev, false) == ICM_20948_STAT_OK, ESP_FAIL, TAG, "wake from sleep failed");
    ESP_RETURN_ON_FALSE(icm20948_low_power(&s_icm_dev, false) == ICM_20948_STAT_OK, ESP_FAIL, TAG, "disable low power failed");

    icm20948_internal_sensor_id_bm sensors = (icm20948_internal_sensor_id_bm)(ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR);
    ESP_RETURN_ON_FALSE(icm20948_set_sample_mode(&s_icm_dev, sensors, SAMPLE_MODE_CONTINUOUS) == ICM_20948_STAT_OK, ESP_FAIL, TAG, "set sample mode failed");

    icm20948_fss_t fss = {.a = GPM_2, .g = DPS_250};
    ESP_RETURN_ON_FALSE(icm20948_set_full_scale(&s_icm_dev, sensors, fss) == ICM_20948_STAT_OK, ESP_FAIL, TAG, "set full scale failed");

    icm20948_dlpcfg_t dlpf_cfg = {.a = ACC_D473BW_N499BW, .g = GYR_D361BW4_N376BW5};
    ESP_RETURN_ON_FALSE(icm20948_set_dlpf_cfg(&s_icm_dev, sensors, dlpf_cfg) == ICM_20948_STAT_OK, ESP_FAIL, TAG, "set DLPF config failed");
    ESP_RETURN_ON_FALSE(icm20948_enable_dlpf(&s_icm_dev, ICM_20948_INTERNAL_ACC, false) == ICM_20948_STAT_OK, ESP_FAIL, TAG, "disable accel DLPF failed");
    ESP_RETURN_ON_FALSE(icm20948_enable_dlpf(&s_icm_dev, ICM_20948_INTERNAL_GYR, false) == ICM_20948_STAT_OK, ESP_FAIL, TAG, "disable gyro DLPF failed");

    ESP_RETURN_ON_ERROR(init_icm20948_dmp(&s_icm_dev), TAG, "failed to init DMP");

    if (!s_reader_task_started) {
        BaseType_t task_ok = xTaskCreate(imu_reader_task, "imu_reader", 4096, NULL, 4, NULL);
        ESP_RETURN_ON_FALSE(task_ok == pdPASS, ESP_ERR_NO_MEM, TAG, "failed to start imu reader task");
        s_reader_task_started = true;
    }

    s_imu_initialized = true;
    return ESP_OK;
}

esp_err_t imu_get_pitch_roll_yaw(float *pitch, float *roll, float *yaw) {
    ESP_RETURN_ON_FALSE(pitch != NULL && roll != NULL, ESP_ERR_INVALID_ARG, TAG, "pitch/roll is null");
    ESP_RETURN_ON_FALSE(s_imu_initialized, ESP_ERR_INVALID_STATE, TAG, "imu not initialized");
    ESP_RETURN_ON_FALSE(s_mutex != NULL, ESP_ERR_INVALID_STATE, TAG, "imu mutex missing");

    if (xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    *pitch = s_pitch_deg;
    *roll = s_roll_deg;
    if (yaw != NULL) {
        *yaw = s_yaw_deg;
    }
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t imu_get_pitch_roll(float *pitch, float *roll) {
    return imu_get_pitch_roll_yaw(pitch, roll, NULL);
}

size_t imu_status_line_write(char *buf, size_t cap) {
    if (buf == NULL || cap == 0) {
        return 0;
    }
    float pitch = 0, roll = 0, yaw = 0;
    if (imu_get_pitch_roll_yaw(&pitch, &roll, &yaw) == ESP_OK) {
        int n =
            snprintf(buf, cap, "P:%.1f R:%.1f Y:%.1f deg", (double)pitch, (double)roll, (double)yaw);
        return (n > 0) ? (size_t)n : 0;
    }
    int n = snprintf(buf, cap, "P/R/Y: --- (IMU off)");
    return (n > 0) ? (size_t)n : 0;
}
