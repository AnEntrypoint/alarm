idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES cjson esp_wifi esp_event esp_netif nvs_flash driver esp_driver_gpio esp_http_client esp-tls esp_timer lwip led_strip)