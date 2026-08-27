idf_component_register(
    SRCS 
        "step_gen_test.c"
    INCLUDE_DIRS 
        "inc"
    REQUIRES log driver step_generator
)