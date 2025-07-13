idf_component_register(SRCS "swd.c" "esp_usb_jtag.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES "DAP" "util"
                    LDFRAGMENTS "noflash.lf"
                    )
