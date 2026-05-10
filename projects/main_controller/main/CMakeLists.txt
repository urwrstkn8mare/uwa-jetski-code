idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash
                       REQUIRES app_state can config control height imu servo_drive status_ui tdisplays3 webui
                       INCLUDE_DIRS ".")
