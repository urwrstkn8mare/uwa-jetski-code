idf_component_register(
    SRCS "dashboard_app.c"
    INCLUDE_DIRS "include"
    REQUIRES can can_ui_bridge dashboard_ui
)
