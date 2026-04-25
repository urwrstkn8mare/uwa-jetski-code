#ifndef __ESP32_MPU9250_I2C_H__
#define __ESP32_MPU9250_I2C_H__

#include "driver/i2c_master.h"

extern i2c_master_dev_handle_t _dmp_i2c_handle;

void esp32_i2c_init(i2c_master_bus_handle_t bus_handle, uint8_t dev_addr);
int esp32_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                     unsigned char length, unsigned char const *data);
int esp32_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                    unsigned char length, unsigned char *data);
void esp32_delay_ms(unsigned long num_ms);
void esp32_get_clock_ms(unsigned long *count);

#endif
