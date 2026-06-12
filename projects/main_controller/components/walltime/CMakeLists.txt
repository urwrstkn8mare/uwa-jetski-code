idf_component_register(
    SRCS "walltime.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer
    PRIV_REQUIRES can
)
