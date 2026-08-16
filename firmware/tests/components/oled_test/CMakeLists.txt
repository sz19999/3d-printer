idf_component_register(
    SRCS 
        "oled_test.c"
    INCLUDE_DIRS 
        "inc"
    REQUIRES log driver i2c_oled
)