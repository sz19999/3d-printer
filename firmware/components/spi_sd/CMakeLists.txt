idf_component_register(
    SRCS 
        "spi_sd.c"
    INCLUDE_DIRS 
        "inc"
    REQUIRES driver fatfs sdmmc
)