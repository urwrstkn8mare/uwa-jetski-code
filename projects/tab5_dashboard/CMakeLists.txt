# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.10)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
add_compile_options("-Wno-attributes")
idf_build_set_property(COMPILE_OPTIONS "-I${CMAKE_SOURCE_DIR}/managed_components/espressif__freetype/freetype/include" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-DFT_ERR_PREFIX=FT_Err_" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-DFT2_BUILD_LIBRARY" APPEND)
idf_build_set_property(MINIMAL_BUILD ON)
project(tab5_dashboard)

# # Link freetype to lvgl component to resolve FT_* symbols
# idf_component_get_property(lvgl_lib lvgl__lvgl COMPONENT_LIB)
# idf_component_get_property(ft_lib espressif__freetype COMPONENT_LIB)
# target_link_libraries(${lvgl_lib} INTERFACE ${ft_lib})

