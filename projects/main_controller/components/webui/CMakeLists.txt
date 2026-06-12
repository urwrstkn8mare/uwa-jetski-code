idf_component_register(SRCS "webui.c"
                       INCLUDE_DIRS "include"
                       EMBED_FILES "index.html" "app.css.gz" "app.js.gz" "live.js.gz" "leaflet.js.gz" "leaflet.css.gz"
                       REQUIRES control datalog encoder_can imu servo_drive walltime esp_app_format esp_http_server esp_wifi esp_event esp_timer)
