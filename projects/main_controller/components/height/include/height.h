#pragma once

#include "esp_err.h"
#include "status_ui.h"
#include <stddef.h>
#include <stdint.h>

esp_err_t height_init(status_write_cb_t status_write, void *status_write_ctx);

esp_err_t height_get_cm(int32_t *height_cm);
