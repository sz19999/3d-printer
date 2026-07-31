#include "config.h"
#include "motion_planner.h"
#include "gcode_parser.h"

#include <math.h>
#include <stdbool.h>

/* 
    handles a single motion struct kinematics. 
    this function gets a raw gcode block and prepares an initial trapezoidal acceleration profile. 
*/
void create_initial_profile(GCodeCommand* gcode_cmd, PlannedMotion* motion) {

    static PointMM current_mm = {0.0f, 0.0f, 0.0f, 0.0f};
    static PointSteps current_steps = {0, 0, 0, 0};
    
    // update target coordinate in millimeters
    PointMM target_mm = current_mm;
    update_target_coordinate(gcode_cmd, &target_mm);

    // compute deltas in mm
    float deltas_mm[4];     // holds dx, dy, dz, de
    compute_deltas_mm(deltas_mm ,&target_mm, &current_mm);

    // compute total vector length
    compute_total_path_length(motion, deltas_mm);

    // compute unit vectors
    compute_unit_vectors(motion, deltas_mm);
    
    // kinematics and speed limit calculations
    static float v_target_mm_s = 100.0f / 60.0f; // default speed 100 mm/min -> mm/s
    compute_profile_velocities(gcode_cmd, motion, &v_target_mm_s);

    // compute max path acceleration
    compute_max_path_acceleration(motion);

    // convert & compute absolute integer step targets
    PointSteps target_steps;
    convert_from_mm_to_steps(&target_steps, &target_mm);
    compute_steps(motion, &target_steps, &current_steps);

    // find master axis steps
    compute_master_axis_steps(motion, deltas_mm);

    // compute step directions
    evaluate_step_directions(motion, &current_steps, &target_steps);

    // advance position tracking for next call
    current_mm = target_mm;
    current_steps = target_steps;
}


/* 
    computes the safe junction velocity between two lines
*/
void compute_junction_velocity(PlannedMotion planned_motions[]) {
    
}


/*
    does a backward pass on the planned motion profiles in the ring buffer, in order to check if the printer can
    decelerate safely based on the given entry velocity, path length and exit velocity of each profile.
*/
void backward_pass(PlannedMotion planned_motions[]) {

}

/*
    does a forward pass on the planned motion profiles in order to check if the printer can
    accelerate to the desired cruise velocity based on the entry speed, path length, and exit velocity of each profile.
*/
void forward_pass(PlannedMotion planned_motions[]) {

}




/* 
    ************************************************
    auxiliary functions of create_initial_profile():
    ************************************************
*/


/*
    scale the cruise speed so each component of the vector doesnt exceed each axis top speed
*/
float limit_velocity(float v_target, float ux, float uy, float uz, float ue) {
    float v_allowed = v_target;     // v_target > 0, it is a scalar

    if (fabsf(ux) > 0.0001f) {
        float vx_limit = MAX_VELOCITY_X / fabsf(ux);     // compute max x axis velocity limit
        if (vx_limit < v_allowed) v_allowed = vx_limit; // clamp cruise speed
    }

    if (fabsf(uy) > 0.0001f) {
        float vy_limit = MAX_VELOCITY_Y / fabsf(uy);     
        if (vy_limit < v_allowed) v_allowed = vy_limit; 
    }

    if (fabsf(uz) > 0.0001f) {
        float vz_limit = MAX_VELOCITY_Z / fabsf(uz);     
        if (vz_limit < v_allowed) v_allowed = vz_limit; 
    }

    if (fabsf(ue) > 0.0001f) {
        float ve_limit = MAX_VELOCITY_E / fabsf(ue);     
        if (ve_limit < v_allowed) v_allowed = ve_limit; 
    }

    return v_allowed;
}

void compute_unit_vectors(PlannedMotion* motion, float deltas_mm[]) {
    float dx_mm = deltas_mm[0];
    float dy_mm = deltas_mm[1];
    float dz_mm = deltas_mm[2];
    float de_mm = deltas_mm[3];
    float total_length = motion->path_length_mm;

    float ux = 0.0f;
    float uy = 0.0f;
    float uz = 0.0f;
    float ue = 0.0f;

    if (total_length > 0.0001f) {
        float len_inv = 1.0f / total_length;  // calculate divison once
        ux = dx_mm * len_inv;
        uy = dy_mm * len_inv;
        uz = dz_mm * len_inv;
        ue = de_mm * len_inv;
    }
    
    motion->unit_vec[0] = ux;
    motion->unit_vec[1] = uy;
    motion->unit_vec[2] = uz;
    motion->unit_vec[3] = ue;
}

void compute_max_path_acceleration(PlannedMotion* motion) {
    float accelerations[4] = {0, 0, 0, 0};
    float ux = motion->unit_vec[0];
    float uy = motion->unit_vec[1];
    float uz = motion->unit_vec[2];
    float ue = motion->unit_vec[3];

    if (fabsf(ux) > 0.0001f) accelerations[0] = MAX_ACCELERATION_X / fabsf(ux);
    if (fabsf(uy) > 0.0001f) accelerations[1] = MAX_ACCELERATION_Y / fabsf(uy);
    if (fabsf(uz) > 0.0001f) accelerations[2] = MAX_ACCELERATION_Z / fabsf(uz);
    if (fabsf(ue) > 0.0001f) accelerations[3] = MAX_ACCELERATION_E / fabsf(ue);

    motion->max_path_acceleration = 1e9f;
    for (uint32_t i = 0; i < NUM_AXES; i++) {
        if (accelerations[i] < motion->max_path_acceleration ) {
            motion->max_path_acceleration = accelerations[i];
        }
    }
}

/*
    computes initial trapezoidal profile velocities: v_start, v_end and v_cruise
*/
void compute_profile_velocities(GCodeCommand* gcode_cmd, PlannedMotion* motion, float* v_target_mm_s) {
    float ux = motion->unit_vec[0];
    float uy = motion->unit_vec[1];
    float uz = motion->unit_vec[2];
    float ue = motion->unit_vec[3];

    if (gcode_cmd->has_F) *v_target_mm_s = (gcode_cmd->F) / 60.0f;       // extract desired feedrate and convert to mm/s
    motion->v_cruise = limit_velocity(*v_target_mm_s, ux, uy, uz, ue);   // find max feasible cruise speed
    motion->v_entry = motion->v_exit = 0;                               // set default enter and exit speeds
}

void update_target_coordinate(GCodeCommand* gcode_cmd, PointMM* target_mm) {
    if (gcode_cmd->has_X) target_mm->x = gcode_cmd->X;
    if (gcode_cmd->has_Y) target_mm->y = gcode_cmd->Y;
    if (gcode_cmd->has_Z) target_mm->z = gcode_cmd->Z;
    if (gcode_cmd->has_E) target_mm->e = gcode_cmd->E;
}

void compute_steps(PlannedMotion* motion, PointSteps* target_steps, PointSteps* current_steps) {
    // compute signed step deltas
    int32_t step_dx = target_steps->x - current_steps->x;
    int32_t step_dy = target_steps->y - current_steps->y;
    int32_t step_dz = target_steps->z - current_steps->z;
    int32_t step_de = target_steps->e - current_steps->e;

    motion->steps[0] = (uint32_t)labs(step_dx);
    motion->steps[1] = (uint32_t)labs(step_dy);
    motion->steps[2] = (uint32_t)labs(step_dz);
    motion->steps[3] = (uint32_t)labs(step_de);
}

void convert_from_mm_to_steps(PointSteps* target_steps, PointMM* target_mm) {
    target_steps->x = (int32_t)lroundf(target_mm->x * STEPS_PER_MM_BELT);
    target_steps->y = (int32_t)lroundf(target_mm->y * STEPS_PER_MM_BELT);
    target_steps->z = (int32_t)lroundf(target_mm->z * STEPS_PER_MM_SCREW);
    target_steps->e = (int32_t)lroundf(target_mm->e * STEPS_PER_MM_GEAR);
}

void evaluate_step_directions(PlannedMotion* motion, PointSteps* current_steps, PointSteps* target_steps) {
    // compute signed step deltas
    int32_t step_dx = target_steps->x - current_steps->x;
    int32_t step_dy = target_steps->y - current_steps->y;
    int32_t step_dz = target_steps->z - current_steps->z;
    int32_t step_de = target_steps->e - current_steps->e;

    // compute steps directions
    motion->dir_bits = 0;
    if (step_dx < 0) motion->dir_bits |= (1 << 0);
    if (step_dy < 0) motion->dir_bits |= (1 << 1);
    if (step_dz < 0) motion->dir_bits |= (1 << 2);
    if (step_de < 0) motion->dir_bits |= (1 << 3);
}

void compute_deltas_mm(float deltas_mm[] ,PointMM* target_mm,PointMM* current_mm) {
    float dx_mm = target_mm->x - current_mm->x;
    float dy_mm = target_mm->y - current_mm->y;
    float dz_mm = target_mm->z - current_mm->z;
    float de_mm = target_mm->e - current_mm->e;

    deltas_mm[0] = dx_mm;
    deltas_mm[1] = dy_mm;
    deltas_mm[2] = dz_mm;
    deltas_mm[3] = de_mm;
}

/*
    computes the cartesian length using pythagoras theorem
*/
float compute_cartesian_length(float deltas_mm[]) {
    float squared_sum = 0;

    for (uint32_t i = 0; i < NUM_AXES - 1; i++) {
        squared_sum += (deltas_mm[i] * deltas_mm[i]);   // excludes extruder delta
    }

    return sqrtf(squared_sum);
}

/*
    computes total effective path length
*/
void compute_total_path_length(PlannedMotion* motion, float deltas_mm[]) {
    float cartesian_length = compute_cartesian_length(deltas_mm);
    float total_length = cartesian_length;
    float de_mm = deltas_mm[3];

    // if move is pure extrusion, use E length
    if (cartesian_length < 0.0001f) {
        total_length = fabsf(de_mm);
    }

    motion->path_length_mm = total_length;
}

/*
    finds the master axis and computes its electrical steps
*/
void compute_master_axis_steps(PlannedMotion* motion, float deltas[]) {
    uint32_t master_steps = 0;
    uint8_t master_axis = 0;    // x - 0, y - 1, z - 2, e - 3; 
    
    for (uint32_t i = 0; i < NUM_AXES; i++) {
        if (fabsf(deltas[i]) > master_steps) {
            master_steps = (uint32_t)lroundf(fabsf(deltas[i]));
            master_axis = i;
        }
    }

    motion->master_axis = master_axis;
    motion->master_steps = master_steps;
}

/*
    computes the velocity scale factor
*/
void compute_master_axis_steps_per_mm(PlannedMotion* motion, float deltas[]) {
    if (motion->path_length_mm > 0.0001f) {
        motion->master_steps_per_mm = motion->master_steps / motion->path_length_mm;
    }
    else {
        motion->master_steps_per_mm = 0.0f;
    }
}


/*
    ***************************
    ring buffer implementation:
    ***************************
*/


void init_buffer(RingBuffer* buffer) {
    buffer->head = 0;
    buffer->tail = 0;
    buffer->count = 0;
}

bool is_empty(const RingBuffer* buffer) {
    return buffer->count == 0;
}

bool is_full(const RingBuffer* buffer) {
    return buffer->count == BUFFER_SIZE;
}

bool append(RingBuffer* buffer, const PlannedMotion* motion) {
    if (is_full(buffer)) return false;

    buffer->arr[buffer->head] = *motion;
    buffer->head = (buffer->head + 1) % BUFFER_SIZE;
    buffer->count += 1;

    return true;
}

void remove(RingBuffer* buffer) {
    if (is_empty(buffer)) return;

    buffer->count = buffer->count - 1;
    buffer->tail = (buffer->tail + 1) % BUFFER_SIZE;
}

PlannedMotion* front(RingBuffer* buffer) {
    if (is_empty(buffer)) return NULL;

    return &(buffer->arr[buffer->tail]);
}

uint8_t count(const RingBuffer* buffer) {
    return buffer->count;
}