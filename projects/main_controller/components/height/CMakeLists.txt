idf_component_register(
    SRCS "height.c"
    INCLUDE_DIRS "include"
    REQUIRES a02yyuw esp_timer status_ui
    PRIV_REQUIRES driver
)
