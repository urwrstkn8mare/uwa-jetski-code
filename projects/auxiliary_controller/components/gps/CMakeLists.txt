idf_component_register(
    SRCS "gps.c"
    INCLUDE_DIRS "include"
    REQUIRES can freertos lwgps status_ui
    PRIV_REQUIRES esp_driver_uart)
