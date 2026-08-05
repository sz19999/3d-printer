
#include <stdio.h>
#include <string.h>
#include "motion_planner.h"

#define CMDS_NUM 4

int main() {
    // move in a 10 by 10 square
    char* raw_gcode_cmds[CMDS_NUM] = { 
                                        "G1 X10.0 Y0.0 Z0.0 F3400",
                                        "G1 X10.0 Y10.0 Z0.0 F3300",
                                        "G1 X0.0 Y10.0 Z0.0 F3200",
                                        "G1 X0.0 Y0.0 Z0.0 F3100"
                                    };
    
    // init queue cmds
    GCodeCommand gcode_queue[CMDS_NUM];
    memset(gcode_queue, 0, sizeof(GCodeCommand) * CMDS_NUM);
    
    // parse gcode cmds
    for (int i = 0; i < CMDS_NUM; i++) {
        bool succeed = parse_command(raw_gcode_cmds[i], &gcode_queue[i]);
        if (!succeed) {
            printf("\nERROR: Failed to parse a gcode command!\n");
            return 1;
        }
    }

    // create and init buffer
    RingBuffer buffer;
    init_buffer(&buffer);


    PlannedMotion motion;
    memset(&motion, 0, sizeof(PlannedMotion));  // init struct
    int j = 0;

    while (j < CMDS_NUM) {
        // create initial profile and append to buffer
        create_initial_profile(&gcode_queue[j], &motion);
        bool succeed = append(&buffer, &motion);
        if (!succeed) {
            printf("\nERROR: Failed to append a motion profile to the ring buffer!\n");
            return 2;
        }

        compute_junction_velocity(&buffer);
        j++;
    }

    // print ring buffer contents
    j = 0;
    while (j < CMDS_NUM) {
        printf("\n~~~ motion block %d ~~~\n", j);
        printf("entry speed: %f\n", buffer.arr[j].v_entry);
        printf("cruise speed: %f\n", buffer.arr[j].v_cruise);
        printf("exit speed: %f\n", buffer.arr[j].v_exit);
        j++;
    }

    printf("\n~~~ TESTS FINISHED SUCCESSFULLY ~~~");

    return 0;
}