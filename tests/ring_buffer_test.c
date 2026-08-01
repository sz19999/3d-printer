
#include <stdio.h>
#include "motion_planner.h"

int main() {
    
    // create and initialize buffer
    RingBuffer buffer[BUFFER_SIZE];
    init_buffer(buffer);

    while(1) {
        // simulate a gcode command struct reception
        GCodeCommand gcode_cmd = {
            .command_letter = 'G',
            .command_number = 1,
            .type = CMD_VALID,
            .has_X = true,
            .has_Y = false,
            .has_Z = false,
            .has_E = false,
            .has_F = true,
            .has_S = false,
            .has_P = false,
            .X = 50.0f,
            .Y = 0,
            .Z = 0,
            .E = 0,
            .F = 100.0f,
            .P = 0,
            .S = 0,
        }; 

        printf("\n=== New iteration ===\n");
        printf("Buffer after init: head=%u tail=%u count=%u\n",
               buffer->head, buffer->tail, buffer->count);

        // create initial acceleration profile struct and push into the buffer
        PlannedMotion motion;
        create_initial_profile(&gcode_cmd, &motion);

        printf("Motion profile:\n");
        printf("  path_length_mm = %.3f\n", motion.path_length_mm);
        printf("  v_cruise = %.3f, v_entry = %.3f, v_exit = %.3f\n",
               motion.v_cruise, motion.v_entry, motion.v_exit);
        printf("  master_axis = %u, master_steps = %u, dir_bits = 0x%02X\n",
               motion.master_axis, motion.master_steps, motion.dir_bits);
        printf("  steps = [%u, %u, %u, %u]\n",
               motion.steps[0], motion.steps[1], motion.steps[2], motion.steps[3]);

        if (append(buffer, &motion)) {
            printf("Append success: head=%u tail=%u count=%u\n",
                   buffer->head, buffer->tail, buffer->count);
        } else {
            printf("Append failed: buffer full\n");
        }

        for (int i = 0; i < 2000000000; i++);  // delay
    }
}