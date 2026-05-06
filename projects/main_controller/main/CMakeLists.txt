idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash
                       REQUIRES app_state can freertos height imu servo_drive lvgl_status_display tdisplays3
                       INCLUDE_DIRS ".")
