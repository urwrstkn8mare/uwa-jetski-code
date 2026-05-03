idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash esp_timer driver esp_driver_ledc imu height can tdisplays3 lvgl
                       INCLUDE_DIRS "")
