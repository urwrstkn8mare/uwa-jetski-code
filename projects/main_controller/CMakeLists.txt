# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_SOURCE_DIR}/../../shared_components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(main_controller)

# cybergear-robotics/icm20948 ships SPI helpers that do not compile clean on ESP-IDF 5.5; this firmware uses I2C only.
if(TARGET __idf_cybergear-robotics__icm20948)
  target_compile_options(__idf_cybergear-robotics__icm20948 PRIVATE -Wno-error=incompatible-pointer-types)
endif()
