cmake_minimum_required(VERSION 3.16)

# Define lwgps options before ESP-IDF discovers components so the managed
# component can see the value during configure.
if(NOT LWGPS_OPTS_FILE)
    if(DEFINED ENV{LWGPS_OPTS_FILE} AND NOT "$ENV{LWGPS_OPTS_FILE}" STREQUAL "")
        set(LWGPS_OPTS_FILE "$ENV{LWGPS_OPTS_FILE}")
    else()
        set(LWGPS_OPTS_FILE "${CMAKE_SOURCE_DIR}/components/gps/lwgps_opts.h")
    endif()
endif()

list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_SOURCE_DIR}/../../shared_components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
project(auxiliary_controller)
