idf_component_register(
    SRCS "rudder_pot.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_adc
    REQUIRES can freertos)
