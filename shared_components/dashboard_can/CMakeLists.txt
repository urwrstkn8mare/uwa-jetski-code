idf_component_register(
    SRCS "dashboard_can.c"
    INCLUDE_DIRS "include"
    REQUIRES can dashboard_ui status_ui
)
