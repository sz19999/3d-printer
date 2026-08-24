idf_component_register(
    SRCS 
        "step_generator.c"
        "../../../core/motion_planner.c"
    INCLUDE_DIRS 
        "inc"
        "../../../core/inc"
        "../../../"
    REQUIRES driver
)