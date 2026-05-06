/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "waveshare_rgb_lcd_port.h"

static const char *TAG = "lcd_port";

#define CH422G_WR_SET_ADDR 0x24
#define CH422G_WR_IO_ADDR  0x38

#define CH422G_IO_LCD_BL   2
#define CH422G_IO_LCD_RST  3

static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_ch422g_set_dev = NULL;
static i2c_master_dev_handle_t s_ch422g_io_dev = NULL;
static uint8_t s_ch422g_io_state = 0xFF;

static esp_err_t ensure_i2c_ready(void)
{
    if (s_i2c_bus != NULL && s_ch422g_set_dev != NULL && s_ch422g_io_dev != NULL) {
        return ESP_OK;
    }

    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 1,
            .allow_pd = 0,
        },
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (err != ESP_OK) {
        return err;
    }

    const i2c_device_config_t set_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CH422G_WR_SET_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .scl_wait_us = I2C_MASTER_TIMEOUT_MS * 1000,
        .flags = {
            .disable_ack_check = 0,
        },
    };

    err = i2c_master_bus_add_device(s_i2c_bus, &set_dev_cfg, &s_ch422g_set_dev);
    if (err != ESP_OK) {
        if (s_i2c_bus != NULL) {
            (void)i2c_del_master_bus(s_i2c_bus);
            s_i2c_bus = NULL;
        }
        return err;
    }

    const i2c_device_config_t io_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CH422G_WR_IO_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .scl_wait_us = I2C_MASTER_TIMEOUT_MS * 1000,
        .flags = {
            .disable_ack_check = 0,
        },
    };

    err = i2c_master_bus_add_device(s_i2c_bus, &io_dev_cfg, &s_ch422g_io_dev);
    if (err != ESP_OK) {
        if (s_ch422g_set_dev != NULL) {
            (void)i2c_master_bus_rm_device(s_ch422g_set_dev);
            s_ch422g_set_dev = NULL;
        }
        if (s_i2c_bus != NULL) {
            (void)i2c_del_master_bus(s_i2c_bus);
            s_i2c_bus = NULL;
        }
        return err;
    }

    return ESP_OK;
}

static esp_err_t ch422g_enable_outputs(void)
{
    uint8_t value = 0x01;
    return i2c_master_transmit(s_ch422g_set_dev, &value, 1, I2C_MASTER_TIMEOUT_MS);
}

static esp_err_t ch422g_write_io(uint8_t value)
{
    s_ch422g_io_state = value;
    return i2c_master_transmit(s_ch422g_io_dev, &value, 1, I2C_MASTER_TIMEOUT_MS);
}

static esp_err_t prepare_board_power(void)
{
    ESP_RETURN_ON_ERROR(ensure_i2c_ready(), TAG, "Failed to initialize I2C for CH422G");

    ESP_RETURN_ON_ERROR(ch422g_enable_outputs(), TAG, "Failed to enable CH422G outputs");

    // Match the Waveshare 5B bring-up sequence: power and reset lines high first.
    ESP_RETURN_ON_ERROR(
        ch422g_write_io((uint8_t)(0xFF | (1U << CH422G_IO_LCD_BL))),
        TAG,
        "Failed to set CH422G outputs high"
    );
    vTaskDelay(pdMS_TO_TICKS(100));

    // Pulse LCD reset low, then restore it high.
    ESP_RETURN_ON_ERROR(ch422g_write_io((uint8_t)(s_ch422g_io_state & ~(1U << CH422G_IO_LCD_RST))), TAG,
                        "Failed to pulse CH422G LCD reset");
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_RETURN_ON_ERROR(ch422g_write_io(0xFF), TAG, "Failed to release CH422G reset lines");
    vTaskDelay(pdMS_TO_TICKS(200));

    return ESP_OK;
}

esp_err_t waveshare_esp32_s3_rgb_lcd_init(esp_lcd_panel_handle_t *panel_out, uint8_t num_fbs)
{
    if (panel_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Install RGB LCD panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_rgb_panel_config_t panel_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .timings = {
            .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
            .h_res = EXAMPLE_LCD_H_RES,
            .v_res = EXAMPLE_LCD_V_RES,
            .hsync_back_porch = 160,
            .hsync_front_porch = 160,
            .hsync_pulse_width = 24,
            .vsync_back_porch = 23,
            .vsync_front_porch = 12,
            .vsync_pulse_width = 2,
            .flags = {
                .pclk_active_neg = 1,
            },
        },
        .data_width = EXAMPLE_RGB_DATA_WIDTH,
        .bits_per_pixel = EXAMPLE_RGB_BIT_PER_PIXEL,
        .num_fbs = num_fbs,
        .bounce_buffer_size_px = EXAMPLE_RGB_BOUNCE_BUFFER_SIZE,
        .sram_trans_align = 4,
        .psram_trans_align = 64,
        .hsync_gpio_num = EXAMPLE_LCD_IO_RGB_HSYNC,
        .vsync_gpio_num = EXAMPLE_LCD_IO_RGB_VSYNC,
        .de_gpio_num = EXAMPLE_LCD_IO_RGB_DE,
        .pclk_gpio_num = EXAMPLE_LCD_IO_RGB_PCLK,
        .disp_gpio_num = EXAMPLE_LCD_IO_RGB_DISP,
        .data_gpio_nums = {
            EXAMPLE_LCD_IO_RGB_DATA0,
            EXAMPLE_LCD_IO_RGB_DATA1,
            EXAMPLE_LCD_IO_RGB_DATA2,
            EXAMPLE_LCD_IO_RGB_DATA3,
            EXAMPLE_LCD_IO_RGB_DATA4,
            EXAMPLE_LCD_IO_RGB_DATA5,
            EXAMPLE_LCD_IO_RGB_DATA6,
            EXAMPLE_LCD_IO_RGB_DATA7,
            EXAMPLE_LCD_IO_RGB_DATA8,
            EXAMPLE_LCD_IO_RGB_DATA9,
            EXAMPLE_LCD_IO_RGB_DATA10,
            EXAMPLE_LCD_IO_RGB_DATA11,
            EXAMPLE_LCD_IO_RGB_DATA12,
            EXAMPLE_LCD_IO_RGB_DATA13,
            EXAMPLE_LCD_IO_RGB_DATA14,
            EXAMPLE_LCD_IO_RGB_DATA15,
        },
        .flags = {
            .fb_in_psram = 1,
        },
    };

    ESP_RETURN_ON_ERROR(esp_lcd_new_rgb_panel(&panel_config, &panel_handle), TAG,
                        "failed to create RGB LCD panel");

    esp_err_t err = prepare_board_power();
    if (err != ESP_OK) {
        (void)esp_lcd_panel_del(panel_handle);
        return err;
    }

    ESP_LOGI(TAG, "Initialize RGB LCD panel");
    err = esp_lcd_panel_init(panel_handle);
    if (err != ESP_OK) {
        (void)esp_lcd_panel_del(panel_handle);
        return err;
    }

    *panel_out = panel_handle;
    return ESP_OK;
}
