idf_component_register(
    SRCS "imu.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES mpu9250_dmp esp_driver_i2c esp_timer
)
