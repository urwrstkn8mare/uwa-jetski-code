idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash
                       REQUIRES app_state can freertos height imu main_panel_ui servo_drive
                       INCLUDE_DIRS "")
