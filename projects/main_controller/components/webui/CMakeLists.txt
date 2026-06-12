idf_component_register(SRCS "webui.c"
                       INCLUDE_DIRS "include"
                       EMBED_FILES "index.html"
                       REQUIRES control datalog encoder_can imu servo_drive walltime esp_app_format esp_http_server esp_wifi esp_event esp_timer)

# Web assets are syntax-checked (JS) and gzipped into the build dir at build
# time, then embedded. webui.c references the _binary_<name>_gz_* symbols,
# which derive from the file name only, so the location change is invisible.
set(assets app.css app.js live.js leaflet.js leaflet.css)
foreach(asset IN LISTS assets)
    set(src ${COMPONENT_DIR}/${asset})
    set(gz ${CMAKE_CURRENT_BINARY_DIR}/${asset}.gz)
    set(check)
    if(asset MATCHES "\\.js$")
        set(check COMMAND node --check ${src})
    endif()
    add_custom_command(OUTPUT ${gz}
        ${check}
        COMMAND sh -c "gzip -9 -n -c '${src}' > '${gz}'"
        DEPENDS ${src}
        COMMENT "webui: gzip ${asset}"
        VERBATIM)
    target_add_binary_data(${COMPONENT_LIB} ${gz} BINARY)
endforeach()
