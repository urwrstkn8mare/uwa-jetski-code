#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "config.h"

typedef uint8_t servo_channel_t;
#define SERVO_CHANNEL_INVALID UINT8_MAX

#define SERVO_MAX_INSTANCES 8

typedef struct {
    bool in_use;
    int gpio;
    bool ready;
    bool simulated;
    bool cal_mode;
    float cmd_deg;
    servo_calibration_t cal;
} servo_info_t;

esp_err_t servo_drive_init_hw(void);

servo_channel_t servo_drive_open(int gpio_num);

esp_err_t servo_drive_close(servo_channel_t h);

void servo_drive_set_degrees(servo_channel_t h, float deg);

void servo_drive_set_raw_us(servo_channel_t h, float pulse_us);

void servo_drive_get_commanded_degrees(servo_channel_t h, float *out_deg);

void servo_drive_set_cal_mode(servo_channel_t h, bool on);

bool servo_drive_is_cal_mode(servo_channel_t h);

void servo_drive_apply_cal(servo_channel_t h, const servo_calibration_t *cal);

void servo_drive_get_cal(servo_channel_t h, servo_calibration_t *out_cal);

bool servo_drive_any_cal_mode(void);

bool servo_drive_all_ready(void);

int servo_drive_get_count(void);

bool servo_drive_get_info_by_index(int idx, servo_info_t *out_info);

void servo_drive_register_change_cb(void (*cb)(int idx));