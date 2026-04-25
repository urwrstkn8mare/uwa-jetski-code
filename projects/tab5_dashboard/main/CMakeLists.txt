idf_component_register(
    SRCS
        "main.c"
        "ui/dashboard_font.c"
    INCLUDE_DIRS
        "."
        "ui"
    REQUIRES
        dashboard_ui 
)

set_source_files_properties(main.c PROPERTIES COMPILE_OPTIONS "-std=gnu23")

# font stuff

set(FONT_REGULAR_URL "https://raw.githubusercontent.com/displaay/Azeret/main/fonts/ttf/AzeretMono-Regular.ttf")
set(FONT_SEMIBOLD_URL "https://raw.githubusercontent.com/displaay/Azeret/main/fonts/ttf/AzeretMono-SemiBold.ttf")
set(FONT_REGULAR_FILE "AzeretMonoRegular.ttf")
set(FONT_SEMIBOLD_FILE "AzeretMonoSemiBold.ttf")
set(FONT_PARTITION "fonts")
set(FONT_ASSETS_DIR "${CMAKE_BINARY_DIR}/font_assets")
set(FONT_INCLUDE_DIR "${CMAKE_BINARY_DIR}/font_assets_include")
set(FONT_URL_MARKER "${CMAKE_BINARY_DIR}/font_assets.url")

file(MAKE_DIRECTORY "${FONT_ASSETS_DIR}")
file(MAKE_DIRECTORY "${FONT_INCLUDE_DIR}")

set(FONT_NEEDS_DOWNLOAD ON)
if(EXISTS "${FONT_ASSETS_DIR}/${FONT_REGULAR_FILE}" AND EXISTS "${FONT_ASSETS_DIR}/${FONT_SEMIBOLD_FILE}" AND EXISTS "${FONT_URL_MARKER}")
    file(READ "${FONT_URL_MARKER}" FONT_CACHED_URL)
    string(STRIP "${FONT_CACHED_URL}" FONT_CACHED_URL)
    if(FONT_CACHED_URL STREQUAL "${FONT_REGULAR_URL}\n${FONT_SEMIBOLD_URL}")
        set(FONT_NEEDS_DOWNLOAD OFF)
    endif()
endif()

if(FONT_NEEDS_DOWNLOAD)
    message(STATUS "Downloading font from ${FONT_REGULAR_URL}")
    file(DOWNLOAD "${FONT_REGULAR_URL}" "${FONT_ASSETS_DIR}/${FONT_REGULAR_FILE}" STATUS FONT_DOWNLOAD_STATUS SHOW_PROGRESS)
    list(GET FONT_DOWNLOAD_STATUS 0 FONT_DOWNLOAD_CODE)
    if(NOT FONT_DOWNLOAD_CODE EQUAL 0)
        list(GET FONT_DOWNLOAD_STATUS 1 FONT_DOWNLOAD_ERROR)
        message(FATAL_ERROR "Failed to download regular font: ${FONT_DOWNLOAD_ERROR}")
    endif()

    message(STATUS "Downloading font from ${FONT_SEMIBOLD_URL}")
    file(DOWNLOAD "${FONT_SEMIBOLD_URL}" "${FONT_ASSETS_DIR}/${FONT_SEMIBOLD_FILE}" STATUS FONT_DOWNLOAD_STATUS SHOW_PROGRESS)
    list(GET FONT_DOWNLOAD_STATUS 0 FONT_DOWNLOAD_CODE)
    if(NOT FONT_DOWNLOAD_CODE EQUAL 0)
        list(GET FONT_DOWNLOAD_STATUS 1 FONT_DOWNLOAD_ERROR)
        message(FATAL_ERROR "Failed to download semibold font: ${FONT_DOWNLOAD_ERROR}")
    endif()

    file(WRITE "${FONT_URL_MARKER}" "${FONT_REGULAR_URL}\n${FONT_SEMIBOLD_URL}")
endif()

spiffs_create_partition_assets(
    ${FONT_PARTITION}
    "${FONT_ASSETS_DIR}"
    FLASH_IN_PROJECT
    MMAP_FILE_SUPPORT_FORMAT ".ttf"
    DEPENDS "${FONT_ASSETS_DIR}/${FONT_REGULAR_FILE}" "${FONT_ASSETS_DIR}/${FONT_SEMIBOLD_FILE}"
    IMPORT_INC_PATH "${FONT_INCLUDE_DIR}"
)

target_include_directories(${COMPONENT_LIB} PRIVATE "${FONT_INCLUDE_DIR}")
add_dependencies(${COMPONENT_LIB} assets_${FONT_PARTITION}_bin)
