#ifndef _ESP32_MPU9250_I2C_H_
#define _ESP32_MPU9250_I2C_H_

#include "driver/i2c_master.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    uint8_t addr;
} esp32_mpu9250_i2c_t;

extern esp32_mpu9250_i2c_t *g_mpu9250_i2c;

int esp32_mpu9250_i2c_init(i2c_master_bus_handle_t bus, uint8_t addr);
int i2c_write(unsigned char slave_addr, unsigned char reg_addr,
              unsigned char length, unsigned char *data);
int i2c_read(unsigned char slave_addr, unsigned char reg_addr,
             unsigned char length, unsigned char *data);
int delay_ms(unsigned long num_ms);
int get_ms(unsigned long *count);

#ifdef __cplusplus
}
#endif

#endif
