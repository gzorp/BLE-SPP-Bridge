set(srcs "ble_spp_server_main.c")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_ringbuf bt
                       PRIV_REQUIRES nvs_flash)
