#include <stdio.h>
#include "gcode_parser.h"

int main(void) {
    char gcode_line[] = "G1 F523 X12.0 Y22.7 Z0.0 ; Move to (12,22.7,0)";
    
    GCodeCommand cmd = {
        .command_letter = ' ',
        .command_number = 0,
        .type = CMD_VALID,
        .has_X = false,
        .has_Y = false,
        .has_Z = false,
        .has_E = false,
        .has_F = false,
        .has_S = false,
        .has_P = false,
        .X = 0,
        .Y = 0,
        .Z = 0,
        .E = 0,
        .F = 0,
        .P = 0,
        .S = 0,
    };

    bool parse_status = parse_command(gcode_line, &cmd);  // this func is being tested

    printf("\ncmd letter: %c\n", cmd.command_letter);
    printf("cmd number: %d\n", cmd.command_number);

    if (parse_status) {
        printf("type: CMD_VALID\n");
    }
    else {
        printf("type: CMD_INVALID\n");
    }

    printf("\ncmd has X: %s\n", cmd.has_X ? "true" : "false");
    printf("X value: %f\n", cmd.X);

    printf("\ncmd has Y: %s\n", cmd.has_Y ? "true" : "false");
    printf("Y value: %f\n", cmd.Y);
    
    printf("\ncmd has Z: %s\n", cmd.has_Z ? "true" : "false");
    printf("Z value: %f\n", cmd.Z);

    printf("\ncmd has E: %s\n", cmd.has_E ? "true" : "false");
    printf("E value: %f\n", cmd.E);    

    printf("\ncmd has F: %s\n", cmd.has_F ? "true" : "false");
    printf("F value: %f\n", cmd.F);

    printf("\ncmd has S: %s\n", cmd.has_S ? "true" : "false");
    printf("S value: %d\n", cmd.S);

    printf("\ncmd has P: %s\n", cmd.has_P ? "true" : "false");
    printf("P value: %d\n", cmd.P);

    printf("\nParser test ended successfully!\n");
    return 0;
}