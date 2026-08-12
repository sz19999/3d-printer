idf_component_register(
    SRCS 
        "main.c"
        "../../core/motion_planner.c"
        "../../core/gcode_parser.c"
        "../../core/gcode_lexer.c"
    INCLUDE_DIRS 
        "."
        "../../core"
        "../../core/inc"
        "../.."
    REQUIRES log
)