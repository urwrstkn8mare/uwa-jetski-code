idf_component_register(
    SRCS "height.c"
    INCLUDE_DIRS "include"
    REQUIRES a02yyuw can esp_timer imu status_ui
    PRIV_REQUIRES driver
)
