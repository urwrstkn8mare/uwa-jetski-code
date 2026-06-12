idf_component_register(
    SRCS "datalog.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer
    PRIV_REQUIRES can control height imu encoder_can servo_drive walltime joltwallet__littlefs
)
