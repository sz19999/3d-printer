
#include <stdio.h>
#include <string.h>
#include "motion_planner.h"

#define CMDS_NUM 4

void print_velocities(RingBuffer*);
void print_profiles(RingBuffer*);

int main() {
    // move in a 10 by 10 square
    char* raw_gcode_cmds[CMDS_NUM] = { 
                                        "G1 X10.0 Y0.0 Z0.0 F3500",
                                        "G1 X10.0 Y1.0 Z0.0 F3500",
                                        "G1 X11.0 Y1.0 Z0.0 F3500",
                                        "G1 X11.0 Y2.0 Z0.0 F3500"
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

        // compute junction velocity
        printf("\n~~~ iteration no. %d ~~~\n", j);
        compute_junction_velocity(&buffer);
        print_velocities(&buffer);
        
        // do forward and backward passes
        backward_pass(&buffer);
        printf("\ndoing a backward pass:\n");
        print_velocities(&buffer);

        forward_pass(&buffer);
        printf("\ndoing a forward pass:\n");
        print_velocities(&buffer);

        recalculate_cruise_speed(&buffer);
        printf("\nrecalculating cruise speed:\n");
        print_velocities(&buffer);

        finalize_motion_profiles(&buffer);
        printf("\nrecalculating cruise speed:\n");
        print_profiles(&buffer);

        j++;
    }

    printf("\n~~~ TESTS FINISHED SUCCESSFULLY ~~~");

    return 0;
}

void print_velocities(RingBuffer* buffer) {
    // print ring buffer contents
    int j = 0;
    while (j < buffer->count) {
        uint8_t idx = (buffer->tail + j) % BUFFER_SIZE;
        printf("\n~~~ motion block %d ~~~\n", j);
        printf("entry speed: %f\n", buffer->arr[idx].v_entry);
        printf("cruise speed: %f\n", buffer->arr[idx].v_cruise);
        printf("exit speed: %f\n", buffer->arr[idx].v_exit);
        j++;
    }
}

void print_profiles(RingBuffer* buffer) {
    // print ring buffer contents
    int j = 0;
    while (j < buffer->count) {
        uint8_t idx = (buffer->tail + j) % BUFFER_SIZE;
        printf("\n~~~ motion block %d ~~~\n", j);
        printf("motion planner parameters:\n");
        printf("path length mm: %.2f\n", buffer->arr[idx].path_length_mm);
        printf("total vector len: %.2f\n", buffer->arr[idx].total_vector_length);
        printf("unit vector: (%.2f, %.2f, %.2f, %.2f)\n", 
            buffer->arr[idx].unit_vec[0],
            buffer->arr[idx].unit_vec[1],
            buffer->arr[idx].unit_vec[2],
            buffer->arr[idx].unit_vec[3]
        );
        printf("cartesian unit vector: (%.2f, %.2f, %.2f)\n", 
            buffer->arr[idx].unit_vec[0],
            buffer->arr[idx].unit_vec[1],
            buffer->arr[idx].unit_vec[2]
        );
        printf("entry speed: %.2f\n", buffer->arr[idx].v_entry);
        printf("cruise speed: %.2f\n", buffer->arr[idx].v_cruise);
        printf("exit speed: %.2f\n", buffer->arr[idx].v_exit);
        printf("max path acceleration: %.2f\n", buffer->arr[idx].max_path_acceleration);
        printf("max vec acceleration: %.2f\n", buffer->arr[idx].max_vector_acceleration);
        printf("dir bits: %x\n", buffer->arr[idx].dir_bits);
        printf("master axis: %d\n", buffer->arr[idx].master_axis);
        printf("master steps: %d\n", buffer->arr[idx].master_steps);
        printf("master steps per mm: %.2f\n", buffer->arr[idx].master_steps_per_mm);
        printf("steps: (%d, %d, %d, %d)\n",
            buffer->arr[idx].steps[0],
            buffer->arr[idx].steps[1],
            buffer->arr[idx].steps[2],
            buffer->arr[idx].steps[3]
        );
        printf("accel steps: %d\n", buffer->arr[idx].accel_steps);
        printf("cruise steps: %d\n", buffer->arr[idx].cruise_steps);
        printf("decel steps: %d\n", buffer->arr[idx].decel_steps);

        j++;
    }
}