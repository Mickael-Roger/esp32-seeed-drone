idf_component_register(
    SRCS
        "main.c"
        "wifi_sta.c"
        "udp_push.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_timer
        esp_wifi
        esp_event
        esp_netif
        esp_http_server
        nvs_flash
        json
        lwip
        icm20948
)
