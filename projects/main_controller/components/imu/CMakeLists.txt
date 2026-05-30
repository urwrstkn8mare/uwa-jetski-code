idf_component_register(
    SRCS "imu.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver esp_common esp_driver_i2c
    REQUIRES can config icm20948 status_ui
)
