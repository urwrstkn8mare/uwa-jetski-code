idf_component_register(SRCS "webui.c"
                       INCLUDE_DIRS "include"
                       EMBED_FILES "index.html"
                       PRIV_REQUIRES nvs_flash
                       REQUIRES config control esp_http_server esp_wifi esp_event)
