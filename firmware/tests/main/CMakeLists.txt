idf_component_register(
    SRCS 
        "test_main.c"
    INCLUDE_DIRS 
        "."
    REQUIRES log driver oled_test i2c_oled
)