idf_component_register(
    SRCS "gps.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_driver_uart esp_timer
    REQUIRES can freertos igrr__libnmea)
