idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash
                       REQUIRES can freertos height imu main_panel_ui servo_drive vehicle_inputs
                       INCLUDE_DIRS "")
