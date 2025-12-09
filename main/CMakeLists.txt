idf_component_register(
  SRCS
    "main.c"
    "simple_wifi.c"
    "http_server.c"
    "tsl2561.c"
    "signal.c"

  INCLUDE_DIRS "."

  REQUIRES
    nvs_flash
    esp_wifi
    esp_event
    esp_netif
    esp_http_server
    driver             # legacy driver (provides driver/i2c.h and driver/gpio.h)
    esp_driver_gpio    # new split driver (also provides driver/gpio.h)
    esp_timer
    freertos
    esp_driver_ledc
    json
)
