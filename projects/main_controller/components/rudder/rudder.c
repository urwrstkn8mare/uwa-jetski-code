#include "rudder.h"

#include "can_ids.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "status_ui.h"
#include <string.h>

static SemaphoreHandle_t s_mx;
static uint16_t s_pct = 50;
static bool s_have;
static TickType_t s_tick;

static void once_init(void) {
    if (s_mx == NULL) {
        s_mx = xSemaphoreCreateMutex();
    }
}

void rudder_on_can_rx(const uint8_t buffer[8], uint32_t header_id) {
    if (buffer == NULL || header_id != CAN_ID_POTENTIOMETER) {
        return;
    }
    once_init();

    uint16_t v;
    memcpy(&v, buffer, sizeof(v));
    xSemaphoreTake(s_mx, portMAX_DELAY);
    s_pct = (v > 100u) ? 100u : v;
    s_have = true;
    s_tick = xTaskGetTickCount();
    xSemaphoreGive(s_mx);
    status_ui_update("Rudder", "CAN %u%%", (unsigned)s_pct);
}

bool rudder_is_fresh(uint32_t max_age_ms, uint16_t *pct_out) {
    once_init();

    xSemaphoreTake(s_mx, portMAX_DELAY);
    const bool have = s_have;
    const uint16_t pct = s_pct;
    const TickType_t last = s_tick;
    xSemaphoreGive(s_mx);

    if (!have) {
        return false;
    }
    const TickType_t max_ticks = pdMS_TO_TICKS(max_age_ms ? max_age_ms : 1);
    if ((xTaskGetTickCount() - last) > max_ticks) {
        return false;
    }
    if (pct_out != NULL) {
        *pct_out = pct;
    }
    return true;
}
