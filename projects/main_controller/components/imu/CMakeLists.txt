idf_component_register(
    SRCS "imu.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver esp_common
    REQUIRES icm20948
)
