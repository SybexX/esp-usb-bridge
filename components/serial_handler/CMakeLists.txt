idf_component_register(SRCS "serial_handler.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES esp-serial-flasher driver freertos esp_timer platform)
