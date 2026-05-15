idf_component_register(
    SRCS "servo_drive.c"
    INCLUDE_DIRS "include"
    REQUIRES config
    PRIV_REQUIRES esp_driver_ledc status_ui)
