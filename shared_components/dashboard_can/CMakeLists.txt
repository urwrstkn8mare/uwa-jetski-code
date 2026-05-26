idf_component_register(
    SRCS "dashboard_can.c"
    INCLUDE_DIRS "include"
    REQUIRES can dashboard_ui encoder_can status_ui
)
