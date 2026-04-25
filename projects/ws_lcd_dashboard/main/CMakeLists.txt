idf_component_register(SRCS "main.c" INCLUDE_DIRS "." REQUIRES dashboard_ui PRIV_REQUIRES ws_display)
idf_component_get_property(lvgl_lib lvgl__lvgl COMPONENT_LIB)
target_compile_options(${lvgl_lib} PRIVATE -Wno-format)
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu23)
