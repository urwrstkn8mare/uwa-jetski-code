#include "dashboard_demo.h"

#include "misc/lv_math.h"

/* 32‑bit amplitude, 16‑bit sine → promotion to 32 bits before division. */
static int32_t sin_scale(uint16_t period_ms, int32_t amplitude, uint32_t elapsed_ms)
{
    return (amplitude * lv_trigo_sin((int16_t)((elapsed_ms / period_ms) % 360)))
           / LV_TRIGO_SIN_MAX;
}

/* Generates smooth sinusoidal test data to exercise every dashboard widget.
 * Replace this entire file when plumbing in real sensor / CAN data. */
void dashboard_demo_fill(dashboard_data_t *data, uint32_t t)
{
    if (data == NULL) {
        return;
    }

    data->speed_kmh       = 45 + sin_scale(20, 35, t);
    data->height_cm       = 25 + sin_scale(35, 20, t);
    data->height_target_cm = 30;
    data->roll_deg        = sin_scale(15, 30, t);
    data->pitch_deg       = sin_scale(12, 25, t);
    data->heading_deg     = (int32_t)((t / 20) % 360);
    data->battery_percent = 60 + sin_scale(40, 30, t);
    data->battery_voltage_v = 36;
    data->battery_current_a = 10 + sin_scale(15, 5, t);
    data->battery_temp_c  = 32;

    data->motor_count = 2;

    data->motor_percent[0]      = 70 + sin_scale(18, 20, t);
    data->motor_percent[1]      = 65 + sin_scale(22, 25, t);
    data->motor_power_kw_x10[0] = 85 + sin_scale(18, 30, t);
    data->motor_power_kw_x10[1] = 80 + sin_scale(22, 35, t);
    data->motor_rpm[0]          = 4500 + sin_scale(18, 800, t);
    data->motor_rpm[1]          = 4200 + sin_scale(22, 900, t);
    data->motor_temp_c[0]       = 40 + sin_scale(28, 10, t);
    data->motor_temp_c[1]       = 42 + sin_scale(32, 8, t);

    data->rudder_deg       = sin_scale(24, 8, t);
    data->elevon_left_deg  = sin_scale(19, 6, t);
    data->elevon_right_deg = -sin_scale(21, 6, t);
}
