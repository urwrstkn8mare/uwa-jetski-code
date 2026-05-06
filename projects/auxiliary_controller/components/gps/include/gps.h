#pragma once

#include <stddef.h>

void gps_init(void);

size_t gps_status_write(char *buf, size_t cap);
