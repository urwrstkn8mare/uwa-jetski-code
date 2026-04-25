#include "esp32_mpu9250_i2c.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

i2c_master_dev_handle_t _dmp_i2c_handle = NULL;

void esp32_i2c_init(i2c_master_bus_handle_t bus_handle, uint8_t dev_addr) {
    if (_dmp_i2c_handle != NULL) return;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = dev_addr,
        .scl_speed_hz = 400000
    };
    i2c_master_bus_add_device(bus_handle, &dev_cfg, &_dmp_i2c_handle);
}

int esp32_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                     unsigned char length, unsigned char const *data) {
    if (_dmp_i2c_handle == NULL) return -1;
    uint8_t *write_buf = (uint8_t *)malloc(length + 1);
    if (!write_buf) return -1;
    write_buf[0] = reg_addr;
    memcpy(write_buf + 1, data, length);
    esp_err_t err = i2c_master_transmit(_dmp_i2c_handle, write_buf, length + 1, 100);
    free(write_buf);
    return (err == ESP_OK) ? 0 : -1;
}

int esp32_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                    unsigned char length, unsigned char *data) {
    if (_dmp_i2c_handle == NULL) return -1;
    esp_err_t err = i2c_master_transmit_receive(_dmp_i2c_handle, &reg_addr, 1, data, length, 100);
    return (err == ESP_OK) ? 0 : -1;
}

void esp32_delay_ms(unsigned long num_ms) {
    vTaskDelay(num_ms / portTICK_PERIOD_MS);
}

void esp32_get_clock_ms(unsigned long *count) {
    *count = (unsigned long)(esp_timer_get_time() / 1000);
}
