idf_component_register(
    SRCS "inv_mpu.c" "inv_mpu_dmp_motion_driver.c" "esp32_mpu9250_i2c.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    PRIV_REQUIRES driver esp_timer
)

# DMP driver has many warnings, suppress them
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-maybe-uninitialized -Wno-unused-but-set-variable -Wno-misleading-indentation -Wno-pointer-sign -Wno-unused-variable)
