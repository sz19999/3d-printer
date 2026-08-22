idf_component_register(
    SRCS 
        "tmc2209_test.c"
    INCLUDE_DIRS 
        "inc"
    REQUIRES log driver tmc2209
)