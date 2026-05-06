idf_component_register(
    SRCS "gps.c"
    INCLUDE_DIRS "include"
    REQUIRES can freertos lwgps
    PRIV_REQUIRES esp_driver_uart)
