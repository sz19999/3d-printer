idf_component_register(
    SRCS 
        "sd_test.c"
    INCLUDE_DIRS 
        "inc"
    REQUIRES log driver spi_sd
)