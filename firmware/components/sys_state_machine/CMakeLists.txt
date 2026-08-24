idf_component_register(
    SRCS 
        "sys_state_machine.c"
    INCLUDE_DIRS 
        "inc"
    REQUIRES driver esp_timer
)