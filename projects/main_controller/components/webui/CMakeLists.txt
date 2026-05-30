idf_component_register(SRCS "webui.c"
                       INCLUDE_DIRS "include"
                       EMBED_FILES "index.html"
                       REQUIRES control imu servo_drive esp_http_server esp_wifi esp_event esp_timer)
