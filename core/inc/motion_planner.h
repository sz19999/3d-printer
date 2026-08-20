#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <stdbool.h>
#include <stdint.h>

#include "gcode_parser.h"

#define NUM_AXES    4   // X, Y, Z, E
#define BUFFER_SIZE 16  // ring buffer size
#define MIN_PLANNER_BLOCKS 3 // min amount of blocks to start dispatching from ring buffer

typedef enum {
    PLANNER_STATE_BUFFERING, // filling lookahead buffer; do not dispatch yet
    PLANNER_STATE_RUNNING    // active execution; stream blocks to step generator
} PlannerState;

// motion block consumed by the Step Generator task
typedef struct {
    // motion planner parameters:
    float path_length_mm;           // total Cartesian path length (mm)
    float total_vector_length;      
    float unit_vec[NUM_AXES];       // direction unit vector (ux, uy, uz, ue)
    float cartesian_unit_vec[NUM_AXES - 1];
    
    float v_cruise;                 // desired cruise velocity (mm/s)
    float v_entry;                  // entry velocity from junction planner (mm/s)
    float v_exit;                   // exit velocity for next block (mm/s)
    
    float max_path_acceleration;    // max path acceleration allowed for this move 
    float max_vector_acceleration;  
    
    // pre-calculated step generator parameters:
    uint8_t dir_bits;           // dir_bits = [-, -, -, -, x_dir, y_dir, z_dir, e_dir]
    uint8_t master_axis;        // axis with the most steps 
    uint32_t steps[NUM_AXES];   // number of steps of each axis -> steps[] = [x_steps, y_steps, z_steps, e_steps]
    uint32_t master_steps;
    float master_steps_per_mm;  // velocity scale factor: (master_steps / path_length_mm)
    
    //uint32_t initial_period;    // v_start pulse period
    //uint32_t cruise_period;
    //uint32_t final_period;

    uint32_t accel_steps;       // how much steps to accelerate
    uint32_t decel_steps;
    uint32_t cruise_steps;

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
    PlannedMotion arr[BUFFER_SIZE];  // a buffer which holds the motion profiles
    uint8_t tail;   // points on the oldest block
    uint8_t head;   // points on the next empty block space
    uint8_t count;  // holds the number of occupied slots in the buffer
} RingBuffer;

// metadata for thermal and auxiliary control (separate from motion buffer)
typedef struct {
    uint16_t extruder_temp_target;  // 0 = disabled, else 0-500°C
    uint16_t bed_temp_target;       // 0 = disabled, else 0-150°C
    uint8_t  fan_speed;             // 0-255 PWM, 0 = off
    bool     temp_changed;          // flag: temperature setpoint changed
    bool     fan_changed;           // flag: fan speed changed
} MotionMetadata;

// main motion planner functions
bool is_motion_command(GCodeCommand* gcode_cmd);
void handle_motion_command(GCodeCommand* gcode_cmd, RingBuffer* buffer, PointMM*, PointSteps*, bool*);
void handle_metadata_command(GCodeCommand* gcode_cmd, MotionMetadata* metadata);

void create_initial_profile(GCodeCommand* gcode_cmd, PlannedMotion* motion, PointMM*, PointSteps*, bool);
void compute_junction_velocity(RingBuffer* buffer);
void backward_pass(RingBuffer* buffer);
void forward_pass(RingBuffer* buffer);
void recalculate_cruise_speed(RingBuffer* buffer);
void finalize_motion_profiles(RingBuffer* buffer);

// handle_motion_command() auxiliary functions
void plan_motion_segment(GCodeCommand* gcode_cmd, RingBuffer* buffer, PointMM*, PointSteps*, bool);
void set_axes_pos(GCodeCommand* gcode_cmd, PointMM* current_mm, PointSteps* current_steps, bool);
void home_axes(RingBuffer* buffer, PointMM* current_mm, PointSteps* current_steps, bool absolute_mode);

// handle_metadata_command() aux funcs
void set_hotend_temp(GCodeCommand* gcode_cmd, MotionMetadata* metadata);
void set_bed_temp(GCodeCommand* gcode_cmd, MotionMetadata* metadata);
void set_fan_speed(GCodeCommand* gcode_cmd, MotionMetadata* metadata);

// create_initial_profile() helper functions
float limit_velocity(float v_target, float ux, float uy, float uz);
void compute_unit_vectors(PlannedMotion* motion, float deltas_mm[]);
void compute_path_and_vector_lengths(PlannedMotion* motion, float deltas_mm[]);
void compute_max_path_and_vector_acceleration(PlannedMotion* motion);
void compute_profile_velocities(GCodeCommand* gcode_cmd, PlannedMotion* motion, float* v_target_mm_s);
void update_target_coordinate(GCodeCommand* gcode_cmd, PointMM* target_mm, bool absolute_mode);
void compute_steps(PlannedMotion* motion, PointSteps* target_steps, PointSteps* current_steps);
void convert_from_mm_to_steps(PointSteps* target_steps, PointMM* target_mm);
void evaluate_step_directions(PlannedMotion* motion, PointSteps* current_steps, PointSteps* target_steps);
void compute_deltas_mm(float deltas_mm[], PointMM* target_mm, PointMM* current_mm);
float compute_cartesian_length(float deltas_mm[]);
void compute_path_and_vector_lengths(PlannedMotion*motion, float deltas_mm[]);
void compute_master_axis_steps(PlannedMotion* motion);
void compute_master_axis_steps_per_mm(PlannedMotion* motion);

// ring buffer API
void init_buffer(RingBuffer* buffer);
bool is_empty(const RingBuffer* buffer);
bool is_full(const RingBuffer* buffer);
bool append(RingBuffer* buffer, const PlannedMotion* motion);
void pop(RingBuffer* buffer);
PlannedMotion* front(RingBuffer* buffer);
uint8_t count(const RingBuffer* buffer);



#endif