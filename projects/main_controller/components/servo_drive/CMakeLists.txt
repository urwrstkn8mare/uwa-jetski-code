idf_component_register(
    SRCS "servo_drive.c"
    INCLUDE_DIRS "include"
    REQUIRES status_ui
    PRIV_REQUIRES esp_driver_ledc)
