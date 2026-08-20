idf_component_register(
    SRCS 
        "test_main.c"
    INCLUDE_DIRS 
        "."
    REQUIRES oled_test sd_test
)