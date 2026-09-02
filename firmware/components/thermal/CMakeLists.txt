idf_component_register(
    SRCS 
        "pid_controller.c"
        "pid_autotune.c"
        "thermal_task.c"
        "heaters_fans.c"
        "adc_filter.c"
        "adc.c"
        "thermistor.c"
    INCLUDE_DIRS 
        "inc"
        "../../../core/inc"
    REQUIRES esp_driver_gpio log esp_adc driver esp_driver_ledc
)