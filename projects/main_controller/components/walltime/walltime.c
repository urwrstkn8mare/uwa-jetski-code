#include "walltime.h"

#include "can.h"
#include "can_ids.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#include <string.h>

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static int64_t s_offset_ms; /* epoch_ms - uptime_ms */
static walltime_source_t s_source = WALLTIME_SOURCE_NONE;

static void time_rx_cb(const uint8_t buffer[8], uint32_t header_id, uint64_t ts) {
    (void)ts;
    if (buffer == NULL || header_id != CAN_ID_GPS_TIME) return;
    can_gps_time_t t;
    memcpy(&t, buffer, sizeof(t));
    walltime_set(t.epoch_s, WALLTIME_SOURCE_GPS);
}

esp_err_t walltime_init(void) {
    return can_register_rx_cb(time_rx_cb);
}

void walltime_set(uint32_t epoch_s, walltime_source_t source) {
    if (epoch_s == 0) return;
    int64_t offset = (int64_t)epoch_s * 1000 - esp_timer_get_time() / 1000;
    portENTER_CRITICAL(&s_mux);
    s_offset_ms = offset;
    s_source = source;
    portEXIT_CRITICAL(&s_mux);
}

bool walltime_get(uint32_t *epoch_s, walltime_source_t *source) {
    int64_t uptime_ms = esp_timer_get_time() / 1000;
    portENTER_CRITICAL(&s_mux);
    walltime_source_t src = s_source;
    int64_t offset = s_offset_ms;
    portEXIT_CRITICAL(&s_mux);
    if (source) *source = src;
    if (epoch_s) *epoch_s = (src == WALLTIME_SOURCE_NONE)
                                ? 0
                                : (uint32_t)((offset + uptime_ms) / 1000);
    return src != WALLTIME_SOURCE_NONE;
}

uint32_t walltime_epoch_at_uptime_ms(int64_t uptime_ms) {
    portENTER_CRITICAL(&s_mux);
    walltime_source_t src = s_source;
    int64_t offset = s_offset_ms;
    portEXIT_CRITICAL(&s_mux);
    if (src == WALLTIME_SOURCE_NONE) return 0;
    return (uint32_t)((offset + uptime_ms) / 1000);
}
