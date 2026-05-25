idf_component_register(
    SRCS "servo_drive.c"
    INCLUDE_DIRS "include"
    REQUIRES can config
    PRIV_REQUIRES esp_driver_ledc status_ui)
