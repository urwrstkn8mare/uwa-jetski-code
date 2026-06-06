idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash
                       REQUIRES can config control datalog encoder_can height imu servo_drive status_ui tdisplays3 webui
                       INCLUDE_DIRS ".")
