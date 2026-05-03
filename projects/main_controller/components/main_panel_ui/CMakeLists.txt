idf_component_register(
    SRCS "main_panel_ui.c"
    INCLUDE_DIRS "include"
    REQUIRES can height imu runtime_health servo_drive tdisplays3 vehicle_inputs)
