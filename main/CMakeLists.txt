idf_component_register(SRCS "gatt_svr.c" "access-control.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES bt nvs_flash)
