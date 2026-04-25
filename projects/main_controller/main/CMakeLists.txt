idf_component_register(SRCS "main.cpp"
                       PRIV_REQUIRES spi_flash esp_driver_i2c mpu9250 espp__filters
                       INCLUDE_DIRS "")
