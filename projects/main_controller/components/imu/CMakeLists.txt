idf_component_register(
    SRCS "imu.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver
    REQUIRES icm20948
)
