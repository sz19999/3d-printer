idf_component_register(
    SRCS 
        "tmc2209.c"
    INCLUDE_DIRS 
        "inc"
    REQUIRES driver esp_driver_uart esp_driver_gpio
)