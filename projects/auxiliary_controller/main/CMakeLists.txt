idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash
                       REQUIRES can freertos gps rudder_pot lvgl_status_display tdisplays3
                       INCLUDE_DIRS ".")
