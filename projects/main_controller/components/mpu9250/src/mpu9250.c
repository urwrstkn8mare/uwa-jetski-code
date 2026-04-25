#include "mpu9250.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TIMEOUT 100
#define ERR_CHECK(x)                                                           \
  {                                                                            \
    int err = (x);                                                             \
    if (err != ESP_OK)                                                         \
      return x;                                                                \
  }

static const char *TAG = "mpu9250";

int mpu9250_begin(mpu9250_t *mpu, const mpu9250_config_t config, int address,
                  i2c_master_bus_handle_t i2c_bus_handle) {
  ESP_LOGD(TAG, "Initiating connection with MPU9250 at address %#02X", address);

  // Check if device exists at address
  ERR_CHECK(i2c_master_probe(i2c_bus_handle, address, TIMEOUT));
  ESP_LOGD(TAG, "Device exists");

  // Connect to slave
  i2c_device_config_t dev_cfg = {.dev_addr_length = I2C_ADDR_BIT_7,
                                 .device_address = address,
                                 .scl_speed_hz = 400000 /* 400kHz */};
  ERR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &mpu->_handle));
  ESP_LOGD(TAG, "Connected succesfully. Resetting...");

  // Reset MPU
  uint8_t pwr_mgmt_1[2] = {
      0x6B,        // PWR_MGMT_1
      1 << 7 |     // H_RESET => Reset the MPU9250
          0 << 6 | // SLEEP
          0 << 5 | // CYCLE
          0 << 4 | // GYRO_STANDBY
          0 << 3 | // PD_PTAT
          1        // CLKSEL => Select best clock source
  };
  ERR_CHECK(i2c_master_transmit(mpu->_handle, pwr_mgmt_1, sizeof(pwr_mgmt_1),
                                TIMEOUT));

  // Configure MPU
  uint8_t pwr_mgmt_2[2] = {
      0x6C,                            // PWR_MGMT_2
      !config.accel_enabled << 5 |     // Accel X
          !config.accel_enabled << 4 | // Accel Y
          !config.accel_enabled << 3 | // Accel Z
          !config.gyro_enabled << 2 |  // Gyro X
          !config.gyro_enabled << 1 |  // Gyro Y
          !config.gyro_enabled         // Gyro Z
  };
  ERR_CHECK(i2c_master_transmit(mpu->_handle, pwr_mgmt_2, sizeof(pwr_mgmt_2),
                                TIMEOUT));

  int dlpf_cfg = 0;
  int fchoice_b = 0b11;
  switch (config.gyro_temp_filter_level) {
  case 0:
    fchoice_b = 0b01;
    break;
  case 1:
    fchoice_b = 0b10;
    break;
  case 2:
  case 3:
  case 4:
  case 5:
  case 6:
  case 7:
  case 8:
  case 9:
    dlpf_cfg = config.gyro_temp_filter_level - 2;
  }
  uint8_t reg_config[2] = {
      0x1A,        // CONFIG
      0 << 3 |     // Disable FSYNC
          dlpf_cfg // DLPF_CFG
  };
  ERR_CHECK(i2c_master_transmit(mpu->_handle, reg_config, sizeof(reg_config),
                                TIMEOUT));

  if (config.gyro_enabled) {
    ESP_LOGD(TAG, "Configuring gyroscope settings");
    uint8_t gyro_config[2] = {
        0x1B,             // GYRO_CONFIG
        0 << 4 | 0 << 3 | // GYRO_FS_SEL => Set scale to 250 degrees per second
            fchoice_b     // FCHOICE_B
    };
    ERR_CHECK(i2c_master_transmit(mpu->_handle, gyro_config,
                                  sizeof(gyro_config), TIMEOUT));
  }
  if (config.accel_enabled) {
    ESP_LOGD(TAG, "Configuring accelerometer settings");
    uint8_t accel_config[2] = {
        0x1C,           // ACCEL_CONFIG
        0 << 4 | 0 << 3 // ACCEL_FS_SEL => Set scale to +-2g
    };
    ERR_CHECK(i2c_master_transmit(mpu->_handle, accel_config,
                                  sizeof(accel_config), TIMEOUT));

    int accel_fchoice_b = 1;
    int accel_dlpf_cfg = 0;
    switch (config.accel_filter_level) {
    case 0:
      accel_fchoice_b = 0;
      break;
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
      accel_dlpf_cfg = config.accel_filter_level - 1;
    }
    uint8_t accel_config_2[2] = {
        0x1D,                  // ACCEL_CONFIG_2
        accel_fchoice_b << 3 | // FCHOICE_B
            accel_dlpf_cfg     // A_DLPFCFG
    };
    ERR_CHECK(i2c_master_transmit(mpu->_handle, accel_config_2,
                                  sizeof(accel_config_2), TIMEOUT));
  }

  uint8_t int_enable[2] = {
      0x38, // INT_ENABLE
      1     // RAW_RDY_EN => Trigger when raw sensor data is ready
  };
  ERR_CHECK(i2c_master_transmit(mpu->_handle, int_enable, sizeof(int_enable),
                                TIMEOUT));

  if (config.mag_enabled) {
    ESP_LOGD(TAG, "Enabling I2C bypass for magnetometer");
    uint8_t int_pin_cfg[2] = {
        0x37, // INT_PIN_CFG
        1 << 1 // BYPASS_EN => Enable I2C bypass for magnetometer
    };
    ERR_CHECK(i2c_master_transmit(mpu->_handle, int_pin_cfg,
                                  sizeof(int_pin_cfg), TIMEOUT));

    vTaskDelay(10 / portTICK_PERIOD_MS);

    i2c_device_config_t mag_dev_cfg = {.dev_addr_length = I2C_ADDR_BIT_7,
                                       .device_address = 0x0C,
                                       .scl_speed_hz = 400000};
    ERR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &mag_dev_cfg,
                                        &mpu->_mag_handle));

    uint8_t who_am_i_reg = 0x00;
    uint8_t who_am_i = 0;
    ERR_CHECK(i2c_master_transmit_receive(mpu->_mag_handle, &who_am_i_reg, 1,
                                          &who_am_i, 1, TIMEOUT));
    ESP_LOGI(TAG, "AK8963 WHO_AM_I = 0x%02X", who_am_i);

    uint8_t cntl2[2] = {0x0C, 0x01};
    ERR_CHECK(i2c_master_transmit(mpu->_mag_handle, cntl2, sizeof(cntl2),
                                  TIMEOUT));
    vTaskDelay(10 / portTICK_PERIOD_MS);

    uint8_t cntl1[2] = {0x0A, 0x16};
    ERR_CHECK(i2c_master_transmit(mpu->_mag_handle, cntl1, sizeof(cntl1),
                                  TIMEOUT));
    vTaskDelay(10 / portTICK_PERIOD_MS);

    uint8_t asa_reg = 0x10;
    uint8_t asa[3];
    ERR_CHECK(i2c_master_transmit_receive(mpu->_mag_handle, &asa_reg, 1, asa, 3,
                                          TIMEOUT));
    mpu->_mag_sensitivity[0] = ((float)asa[0] - 128) / 256 + 1;
    mpu->_mag_sensitivity[1] = ((float)asa[1] - 128) / 256 + 1;
    mpu->_mag_sensitivity[2] = ((float)asa[2] - 128) / 256 + 1;
    ESP_LOGD(TAG, "Mag sensitivity: X=%.3f Y=%.3f Z=%.3f",
             mpu->_mag_sensitivity[0], mpu->_mag_sensitivity[1],
             mpu->_mag_sensitivity[2]);
  }

  ESP_LOGD(TAG, "Configuration complete");
  mpu->config = config;
  return 0;
}

int mpu9250_update(mpu9250_t *mpu) {
  static uint8_t int_status = 0x3A;
  uint8_t result = 0;
  ERR_CHECK(i2c_master_transmit_receive(mpu->_handle, &int_status, 1, &result,
                                        1, TIMEOUT));
  ESP_LOGD(TAG, "INT_STATUS: %d", result);
  // First bit of INT_STATUS indicates if sensor data has changed
  if (result & 1) {
    if (mpu->config.accel_enabled) {
      uint8_t read_buffer[2] = {0};

      const uint8_t accel_xout_h = 0x3B;
      const uint8_t accel_xout_l = 0x3C;
      ERR_CHECK(i2c_master_transmit_receive(mpu->_handle, &accel_xout_h, 1,
                                            read_buffer, 1, TIMEOUT));
      ERR_CHECK(i2c_master_transmit_receive(mpu->_handle, &accel_xout_l, 1,
                                            read_buffer + 1, 1, TIMEOUT));
      mpu->accel.x = read_buffer[1] | read_buffer[0] << 8;

      const uint8_t accel_yout_h = 0x3D;
      const uint8_t accel_yout_l = 0x3E;
      ERR_CHECK(i2c_master_transmit_receive(mpu->_handle, &accel_yout_h, 1,
                                            read_buffer, 1, TIMEOUT));
      ERR_CHECK(i2c_master_transmit_receive(mpu->_handle, &accel_yout_l, 1,
                                            read_buffer + 1, 1, TIMEOUT));
      mpu->accel.y = read_buffer[1] | read_buffer[0] << 8;

      const uint8_t accel_zout_h = 0x3F;
      const uint8_t accel_zout_l = 0x40;
      ERR_CHECK(i2c_master_transmit_receive(mpu->_handle, &accel_zout_h, 1,
                                            read_buffer, 1, TIMEOUT));
      ERR_CHECK(i2c_master_transmit_receive(mpu->_handle, &accel_zout_l, 1,
                                            read_buffer + 1, 1, TIMEOUT));
      mpu->accel.z = read_buffer[1] | read_buffer[0] << 8;
    }
    if (mpu->config.temp_enabled) {
      uint8_t read_buffer[2] = {0};

      const uint8_t temp_out_h = 0x41;
      const uint8_t temp_out_l = 0x42;
      ERR_CHECK(i2c_master_transmit_receive(mpu->_handle, &temp_out_h, 1,
                                            read_buffer, 1, TIMEOUT));
      ERR_CHECK(i2c_master_transmit_receive(mpu->_handle, &temp_out_l, 1,
                                            read_buffer + 1, 1, TIMEOUT));
      mpu->temp = (read_buffer[1] | read_buffer[0] << 8) / 333.87 + 21;
    }
    if (mpu->config.gyro_enabled) {
      uint8_t read_buffer[2] = {0};

      const uint8_t gyro_xout_h = 0x43;
      const uint8_t gyro_xout_l = 0x44;
      ERR_CHECK(i2c_master_transmit_receive(mpu->_handle, &gyro_xout_h, 1,
                                            read_buffer, 1, TIMEOUT));
      ERR_CHECK(i2c_master_transmit_receive(mpu->_handle, &gyro_xout_l, 1,
                                            read_buffer + 1, 1, TIMEOUT));
      mpu->gyro.x = read_buffer[1] | read_buffer[0] << 8;

      const uint8_t gyro_yout_h = 0x45;
      const uint8_t gyro_yout_l = 0x46;
      ERR_CHECK(i2c_master_transmit_receive(mpu->_handle, &gyro_yout_h, 1,
                                            read_buffer, 1, TIMEOUT));
      ERR_CHECK(i2c_master_transmit_receive(mpu->_handle, &gyro_yout_l, 1,
                                            read_buffer + 1, 1, TIMEOUT));
      mpu->gyro.y = read_buffer[1] | read_buffer[0] << 8;

      const uint8_t gyro_zout_h = 0x47;
      const uint8_t gyro_zout_l = 0x48;
      ERR_CHECK(i2c_master_transmit_receive(mpu->_handle, &gyro_zout_h, 1,
                                            read_buffer, 1, TIMEOUT));
      ERR_CHECK(i2c_master_transmit_receive(mpu->_handle, &gyro_zout_l, 1,
                                            read_buffer + 1, 1, TIMEOUT));
      mpu->gyro.z = read_buffer[1] | read_buffer[0] << 8;
    }
    if (mpu->config.mag_enabled) {
      uint8_t status1;
      const uint8_t st1_reg = 0x02;
      ERR_CHECK(i2c_master_transmit_receive(mpu->_mag_handle, &st1_reg, 1,
                                            &status1, 1, TIMEOUT));
      if (status1 & 0x01) {
        uint8_t mag_data[8];
        const uint8_t hxl_reg = 0x03;
        ERR_CHECK(i2c_master_transmit_receive(mpu->_mag_handle, &hxl_reg, 1,
                                              mag_data, 8, TIMEOUT));
        mpu->mag.x = (int16_t)(mag_data[1] << 8 | mag_data[0]);
        mpu->mag.y = (int16_t)(mag_data[3] << 8 | mag_data[2]);
        mpu->mag.z = (int16_t)(mag_data[5] << 8 | mag_data[4]);
        uint8_t st2 = mag_data[7];
        (void)st2;
      }
    }
    return 1;
  }

  return 0;
}
