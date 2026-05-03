idf_component_register(
    SRCS "gps.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_driver_uart
    REQUIRES can freertos)
