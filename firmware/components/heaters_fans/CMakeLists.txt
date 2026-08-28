idf_component_register(
    SRCS 
        "heaters_fans.c"
    INCLUDE_DIRS 
        "inc"
    REQUIRES driver esp_driver_gpio
)