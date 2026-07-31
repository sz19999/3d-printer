#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <stdint.h>

#define NUM_AXES    4   // X, Y, Z, E
#define BUFFER_SIZE 16

// Motion block consumed by the Step Generator task
typedef struct {
    // motion planner parameters
    float path_length_mm;           // total Cartesian path length (mm)
    float unit_vec[NUM_AXES];       // direction unit vector (ux, uy, uz, ue)
    float master_steps_per_mm;      // velocity scale factor: (master_steps / path_length_mm)
    
    float v_cruise;                 // desired cruise velocity (mm/s)
    float v_entry;                  // entry velocity from junction planner (mm/s)
    float v_exit;                   // exit velocity for next block (mm/s)
    
    float max_path_acceleration;    // max path acceleration allowed for this move 
    
    // pre-calculated step generator parameters
    uint8_t dir_bits;           // dir_bits = [-, -, -, -, x_dir, y_dir, z_dir, e_dir]
    uint8_t master_axis;        // axis with the most steps 
    uint32_t steps[NUM_AXES];   // number of steps of each axis -> steps[] = [x_steps, y_steps, z_steps, e_steps]
    uint32_t master_steps;

    uint32_t initial_period;    // v_start pulse period
    uint32_t cruise_period;
    uint32_t final_period;

    uint32_t accel_steps;       // how much steps to accelerate
    uint32_t decel_steps;
    int32_t  accel_rate_factor; // how much consecutive pulses periods differ
    int32_t  decel_rate_factor;

    // additional 
    uint16_t extruder_temp_target;  // 0 = 0C, 5000 = 500C (max)
    uint16_t bed_temp_target;
    uint8_t  fan_speed;             // 0 = stop, 255 = full speed

} PlannedMotion;

typedef struct 
{
    int32_t x, y, z, e;
} PointSteps;

typedef struct 
{
    float x, y, z, e;
} PointMM;

typedef struct {
    PlannedMotion buffer[BUFFER_SIZE];
    uint8_t tail;
    uint8_t head;
    uint8_t count;
} ;


#endif