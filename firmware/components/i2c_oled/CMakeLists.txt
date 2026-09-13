idf_component_register(
    SRCS 
        "i2c_oled.c"
        "user_interface.c"
    INCLUDE_DIRS 
        "inc"
    REQUIRES driver esp_driver_i2c sys_state_machine
)