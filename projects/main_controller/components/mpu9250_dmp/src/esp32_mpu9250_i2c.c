#include "esp32_mpu9250_i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "mpu9250_dmp";

esp32_mpu9250_i2c_t *g_mpu9250_i2c = NULL;

int esp32_mpu9250_i2c_init(i2c_master_bus_handle_t bus, uint8_t addr)
{
    if (g_mpu9250_i2c != NULL) {
        return 0;
    }
    g_mpu9250_i2c = (esp32_mpu9250_i2c_t *)malloc(sizeof(esp32_mpu9250_i2c_t));
    if (g_mpu9250_i2c == NULL) {
        return -1;
    }
    g_mpu9250_i2c->bus_handle = bus;
    g_mpu9250_i2c->addr = addr;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };

    if (i2c_master_bus_add_device(bus, &dev_cfg, &g_mpu9250_i2c->dev_handle) != ESP_OK) {
        free(g_mpu9250_i2c);
        g_mpu9250_i2c = NULL;
        return -1;
    }
    return 0;
}

int i2c_write(unsigned char slave_addr, unsigned char reg_addr,
              unsigned char length, unsigned char *data)
{
    if (g_mpu9250_i2c == NULL) {
        return -1;
    }
    // The DMP driver expects to write a register address followed by data
    // ESP-IDF's i2c_master_transmit can do this in one call
    uint8_t *write_buf = (uint8_t *)malloc(length + 1);
    if (write_buf == NULL) {
        return -1;
    }
    write_buf[0] = reg_addr;
    memcpy(write_buf + 1, data, length);

    esp_err_t err = i2c_master_transmit(g_mpu9250_i2c->dev_handle, write_buf, length + 1, 100);
    free(write_buf);
    return (err == ESP_OK) ? 0 : -1;
}

int i2c_read(unsigned char slave_addr, unsigned char reg_addr,
             unsigned char length, unsigned char *data)
{
    if (g_mpu9250_i2c == NULL) {
        return -1;
    }
    esp_err_t err = i2c_master_transmit_receive(g_mpu9250_i2c->dev_handle,
                                                 &reg_addr, 1,
                                                 data, length, 100);
    return (err == ESP_OK) ? 0 : -1;
}

int delay_ms(unsigned long num_ms)
{
    vTaskDelay(num_ms / portTICK_PERIOD_MS);
    return 0;
}

int get_ms(unsigned long *count)
{
    if (count == NULL) {
        return -1;
    }
    *count = (unsigned long)(esp_timer_get_time() / 1000);
    return 0;
}
