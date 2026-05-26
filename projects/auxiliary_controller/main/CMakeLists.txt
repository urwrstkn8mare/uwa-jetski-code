idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash
                       REQUIRES can freertos gps joystick status_ui tdisplays3
                       INCLUDE_DIRS ".")
