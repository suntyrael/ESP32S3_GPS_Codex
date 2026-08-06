idf_component_register(
    SRCS
        "app_main.c"
        "i2c_bus.c"
        "lsm6dsr.c"
        "lis2mdl.c"
        "bmp388.c"
        "battery.c"
        "sensors.c"
        "diagnostics.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_gpio
        esp_driver_i2c
        esp_adc
        esp_timer
        nvs_flash
)
