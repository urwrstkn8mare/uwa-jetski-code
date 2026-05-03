idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash esp_adc esp_driver_uart driver esp_timer
                       REQUIRES can tdisplays3 lvgl
                       INCLUDE_DIRS ".")
