idf_component_register(
    SRCS 
        "pid_controller.c"
        "pid_autotune.c"
        "thermal_task.c"
        "heaters_fans.c"
        "adc_filter.c"
    INCLUDE_DIRS 
        "inc"
    REQUIRES esp_driver_gpio log esp_adc
)