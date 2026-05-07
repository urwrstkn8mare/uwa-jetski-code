idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash
                       REQUIRES app_state can freertos height imu servo_drive status_ui tdisplays3
                       INCLUDE_DIRS ".")
