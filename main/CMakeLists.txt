idf_component_register(SRCS "main.c" "espnow_client.c" "nvs_helper.c"
                    PRIV_REQUIRES nvs_flash esp_event esp_netif esp_wifi esp_driver_gpio esp_timer json
                    INCLUDE_DIRS ".")
