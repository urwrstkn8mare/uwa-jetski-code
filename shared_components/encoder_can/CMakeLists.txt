idf_component_register(
    SRCS "encoder_can.c"
    INCLUDE_DIRS "include"
    REQUIRES can esp_timer freertos status_ui
)
