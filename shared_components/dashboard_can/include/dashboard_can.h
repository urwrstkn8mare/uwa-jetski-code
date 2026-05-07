#pragma once

#include <stddef.h>
#include <stdint.h>

#include "dashboard_ui.h"
#include "esp_err.h"
#include "lvgl.h"

typedef esp_err_t (*dashboard_can_lock_fn_t)(int32_t timeout_ms, void *ctx);
typedef void (*dashboard_can_unlock_fn_t)(void *ctx);

esp_err_t dashboard_can_attach(dashboard_can_lock_fn_t lock, dashboard_can_unlock_fn_t unlock,
                               void *lock_ctx);

esp_err_t dashboard_can_start(void);
