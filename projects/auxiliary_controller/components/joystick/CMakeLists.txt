idf_component_register(
    SRCS "joystick.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_adc
    REQUIRES can freertos status_ui)
