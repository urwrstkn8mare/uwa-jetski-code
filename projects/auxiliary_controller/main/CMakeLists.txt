idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash
                       REQUIRES aux_panel_ui can gps rudder_pot
                       INCLUDE_DIRS ".")
