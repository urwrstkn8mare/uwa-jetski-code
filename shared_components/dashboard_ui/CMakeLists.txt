idf_component_register(
    SRCS
        "dashboard_ui.c"
        "dashboard_demo.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        lvgl
)

set_source_files_properties(dashboard_ui.c PROPERTIES COMPILE_OPTIONS "-std=gnu23")
