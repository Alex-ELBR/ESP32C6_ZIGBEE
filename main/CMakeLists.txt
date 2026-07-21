idf_component_register(
    SRCS "main.c" "esp_zigbee_gateway.c"
    INCLUDE_DIRS "."
    REQUIRES protocol_examples_common espressif__esp-zigbee-lib espressif__ssd1306 esp_wifi esp_event nvs_flash
)
