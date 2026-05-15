#pragma once

#include "esp_err.h"

esp_err_t webui_start(void);
void webui_stop(void);

void webui_notify_servo_change(int handle);
void webui_notify_cal_mode_change(int handle);
void webui_notify_armed(void);
void webui_notify_target(void);
