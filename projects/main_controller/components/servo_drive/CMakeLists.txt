idf_component_register(
    SRCS "servo_drive.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_driver_ledc)
