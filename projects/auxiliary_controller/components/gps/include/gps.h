#pragma once

#include "status_ui.h"
#include <stddef.h>

void gps_init(status_write_cb_t status_write, void *status_write_ctx);
