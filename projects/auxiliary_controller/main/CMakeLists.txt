idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash
                       REQUIRES can freertos gps rudder_pot status_ui tdisplays3
                       INCLUDE_DIRS ".")
