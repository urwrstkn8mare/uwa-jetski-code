idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash
                       REQUIRES can config control height imu rudder servo_drive status_ui tdisplays3 webui
                       INCLUDE_DIRS ".")
