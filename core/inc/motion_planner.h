#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <stdint.h>

#define NUM_AXES 4 // X, Y, Z, E

// Motion block consumed by the Step Generator task
typedef struct {
    uint8_t dir_bits;           // dir_bits = [-, -, -, -, x_dir, y_dir, z_dir, e_dir]
    uint8_t master_axis;        // axis with the most steps

    uint32_t steps[NUM_AXES];   // number of steps of each axis -> steps[] = [x_steps, y_steps, z_steps, e_steps]
    uint32_t master_steps;

    uint32_t initial_period;    // v_start pulse period
    uint32_t nominal_period;
    uint32_t final_period;

    uint32_t accel_steps;       // how much steps to accelerate
    uint32_t decel_steps;
    int32_t  accel_rate_factor; // how much consecutive pulses periods differ
    int32_t  decel_rate_factor;

    uint16_t extruder_temp_target;  // 0 = 0C, 5000 = 500C (max)
    uint16_t bed_temp_target;
    uint8_t  fan_speed;             // 0 = stop, 255 = full speed

} PlannedMotion;

typedef struct 
{
    float x;
    float y;
    float z;
    float e;
} Point;




#endif