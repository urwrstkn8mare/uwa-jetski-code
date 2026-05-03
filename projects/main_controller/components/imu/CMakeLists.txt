idf_component_register(
    SRCS "imu.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver esp_timer
    REQUIRES cybergear-robotics__icm20948
)
