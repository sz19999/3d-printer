idf_component_register(
    SRCS 
        "tmc2209_test.c"
    INCLUDE_DIRS 
        "inc"
    REQUIRES driver tmc2209
)