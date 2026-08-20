#include "config.h"
#include "motion_planner.h"
#include "gcode_parser.h"
#include "esp_log.h"

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h> // for memset func

/* 
    checks if a G-code command is a motion command (G0, G1, G2, G3)
*/
bool is_motion_command(GCodeCommand* gcode_cmd) {
    if (gcode_cmd->command_letter != 'G') return false;
    
    switch (gcode_cmd->command_number) {
        case 0:   // G0
        case 1:   
        case 28: 
        case 90:  
        case 91:  
        case 92:
            return true;
        default:
            return false;
    }
}

/*
    if command is of type 'G', process the command in the motion pipeline.
    compute kinematic parameters, etc.
*/
void handle_motion_command(GCodeCommand* gcode_cmd, RingBuffer* buffer, PointMM* current_mm, PointSteps* current_steps, bool* absolute_mode) {
    if (gcode_cmd->command_letter == 'G') {
        switch (gcode_cmd->command_number) {
            case 0:
                // rapid move with no extrusion
            case 1:
                // regular move with extrusion
                plan_motion_segment(gcode_cmd, buffer, current_mm, current_steps, *absolute_mode);
                break;
            case 92:
                // update axes position variables
                set_axes_pos(gcode_cmd, current_mm, current_steps, *absolute_mode);
                break;
            case 28:
                // home axes
                home_axes(buffer, current_mm, current_steps, absolute_mode);
                break;
            case 90:
                // absolute mode
                *absolute_mode = true;
                break;
            case 91:
                // relative move
                *absolute_mode = false;
                break;
            default:
                break;
        }
    }
}

/*
    ***************************************
    handle motion command auxiliary functions:
    ***************************************
*/

void plan_motion_segment(GCodeCommand* gcode_cmd, RingBuffer* buffer, PointMM* current_mm,PointSteps* current_steps, bool absolute_mode) {
    PlannedMotion motion;

    // create base motion profile from command parameters
    create_initial_profile(gcode_cmd, &motion, current_mm, current_steps, absolute_mode);

    // append new segment to lookahead ring buffer
    append(buffer, &motion);

    // calculate maximum junction speed with previous block
    compute_junction_velocity(buffer);

    // lookahead planner passes (deceleration and acceleration constraints)
    backward_pass(buffer);
    forward_pass(buffer);

    // recalculate cruise to a feasible one if needed
    recalculate_cruise_speed(buffer);

    // finalize trapezoid parameters
    finalize_motion_profiles(buffer);
}

void set_axes_pos(GCodeCommand* gcode_cmd, PointMM* current_mm, PointSteps* current_steps, bool absolute_mode) {
    update_target_coordinate(gcode_cmd, current_mm, absolute_mode);
    convert_from_mm_to_steps(current_steps, current_mm);
}

void home_axes(RingBuffer* buffer, PointMM* current_mm, PointSteps* current_steps, bool absolute_mode) {
    GCodeCommand gcode_cmd;

    char* relative_cmd = "G91";
    char* absolute_cmd = "G92";
    char move_cmd[128];
    
    // change to relative mode
    parse_command(relative_cmd, &gcode_cmd);
    handle_motion_command(&gcode_cmd, buffer, current_mm, current_steps, &absolute_mode);
    
    ESP_LOGI("Home Axes", "G-Code command: \"%s\".", relative_cmd);

    // home X axis
    sprintf(move_cmd, "G0 X-%.2f F600", MAX_DISTANCE_X);
    parse_command(move_cmd, &gcode_cmd);
    handle_motion_command(&gcode_cmd, buffer, current_mm, current_steps, &absolute_mode);
    ESP_LOGI("Home Axes", "G-Code command: \"%s\".", move_cmd);

    // home Y axis
    sprintf(move_cmd, "G0 Y-%.2f F600", MAX_DISTANCE_Y);
    parse_command(move_cmd, &gcode_cmd);
    handle_motion_command(&gcode_cmd, buffer, current_mm, current_steps, &absolute_mode);
    ESP_LOGI("Home Axes", "G-Code command: \"%s\".", move_cmd);

    // home Z axis
    sprintf(move_cmd, "G0 Z-%.2f F100", MAX_DISTANCE_Z);
    parse_command(move_cmd, &gcode_cmd);
    handle_motion_command(&gcode_cmd, buffer, current_mm, current_steps, &absolute_mode);
    ESP_LOGI("Home Axes", "G-Code command: \"%s\".", move_cmd);

    // restore to absolute mode
    parse_command(absolute_cmd, &gcode_cmd);
    handle_motion_command(&gcode_cmd, buffer, current_mm, current_steps, &absolute_mode);
    ESP_LOGI("Home Axes", "G-Code command: \"%s\".", absolute_cmd);
}

/*
    ***************************************
    end of handle motion command auxiliary functions
    ***************************************
*/

/* 
    extracts thermal and auxiliary metadata from a G-code command and updates state
    M104 S210 = set hotend temperature
    M140 S60  = set bed temperature
    M106 S200 = set fan speed (0-255)
*/
void handle_metadata_command(GCodeCommand* gcode_cmd, MotionMetadata* metadata) {
    if (gcode_cmd->command_letter == 'M') {
        switch (gcode_cmd->command_number) {
            case 104:  // Set hotend temperature
            case 109:  // Set hotend temperature and wait
                set_hotend_temp(gcode_cmd, metadata);
                break;
            case 140:  // Set bed temperature
            case 190:  // Set bed temperature and wait
                set_bed_temp(gcode_cmd, metadata);
                break;
            case 106:  // Set fan speed
            case 107:  // turn off fan
                set_fan_speed(gcode_cmd, metadata);
                break;
            default:
                break;
        }
    }
}

/*
    ***************************************
    handle metadata command auxiliary functions:
    ***************************************
*/

void set_hotend_temp(GCodeCommand* gcode_cmd, MotionMetadata* metadata) {
    if (gcode_cmd->has_S) {
        if (metadata->extruder_temp_target != gcode_cmd->S) {
            metadata->extruder_temp_target = gcode_cmd->S;
            metadata->temp_changed = true;
        }
    }
}

void set_bed_temp(GCodeCommand* gcode_cmd, MotionMetadata* metadata) {
    if (gcode_cmd->has_S) {
        if (metadata->bed_temp_target != gcode_cmd->S) {
            metadata->bed_temp_target = gcode_cmd->S;
            metadata->temp_changed = true;
        }
    }
}

void set_fan_speed(GCodeCommand* gcode_cmd, MotionMetadata* metadata) {
    if (gcode_cmd->has_S) {
        if (metadata->fan_speed != (uint8_t)gcode_cmd->S) {
            metadata->fan_speed = (uint8_t)gcode_cmd->S;
            metadata->fan_changed = true;
        }
    }
}

/*
    ***************************************
    end of handle metadata command auxiliary functions
    ***************************************
*/

/* 
    handles a single motion struct kinematics. 
    this function gets a raw gcode block and prepares an initial trapezoidal acceleration profile. 
    only processes motion commands (G0, G1, G2, G3)
*/
void create_initial_profile(GCodeCommand* gcode_cmd, PlannedMotion* motion, PointMM* current_mm, PointSteps* current_steps, bool absolute_mode) {

    // initialize all PlannedMotion struct fields
    memset(motion, 0, sizeof(PlannedMotion));
    
    // update target coordinate in millimeters
    PointMM target_mm = *current_mm;
    update_target_coordinate(gcode_cmd, &target_mm, absolute_mode);

    // compute deltas in mm
    float deltas_mm[4];     // holds dx, dy, dz, de
    compute_deltas_mm(deltas_mm ,&target_mm, current_mm);
    
    // compute total vector length & path length
    compute_path_and_vector_lengths(motion, deltas_mm);

    // compute unit vectors
    compute_unit_vectors(motion, deltas_mm);
    
    // kinematics and speed limit calculations
    static float active_feedrate = 3000.0f / 60.0f;  // default speed 3000 mm/min -> mm/s
    compute_profile_velocities(gcode_cmd, motion, &active_feedrate);

    // compute max path and vector accelerations
    compute_max_path_and_vector_acceleration(motion);

    // convert & compute absolute integer step targets
    PointSteps target_steps;
    convert_from_mm_to_steps(&target_steps, &target_mm);
    compute_steps(motion, &target_steps, current_steps);

    // find master axis steps
    compute_master_axis_steps(motion);

    // compute step directions
    evaluate_step_directions(motion, current_steps, &target_steps);

    // compute master axis steps per mm
    compute_master_axis_steps_per_mm(motion);

    // advance position tracking for next call
    *current_mm = target_mm;
    *current_steps = target_steps;
}


/* 
    computes a safe junction velocity between two lines.
    an image of the calculations is provided in the assets folder
*/
void compute_junction_velocity(RingBuffer* buffer) {
    // check if there're at least two motion structs, if not, return
    if (buffer->count < 2) return;

    // else, calc junction velocity:
    
    // do a dot product between the current and the next lines unit vectors.
    // this gives us the cosine of the angle between the two line vectors.
    
    // extract the indexes of the last two inserted motion profiles
    uint8_t curr_idx = (buffer->head + BUFFER_SIZE - 1) % BUFFER_SIZE;
    uint8_t prev_idx = (curr_idx + BUFFER_SIZE - 1) % BUFFER_SIZE;

    // extract N and N-1 motion profiles cartesian unit vectors
    float ux1 = buffer->arr[prev_idx].cartesian_unit_vec[0];
    float uy1 = buffer->arr[prev_idx].cartesian_unit_vec[1];
    float uz1 = buffer->arr[prev_idx].cartesian_unit_vec[2];

    float ux2 = buffer->arr[curr_idx].cartesian_unit_vec[0];
    float uy2 = buffer->arr[curr_idx].cartesian_unit_vec[1];
    float uz2 = buffer->arr[curr_idx].cartesian_unit_vec[2];

    // do the dot product
    // phi is the angle between the vectors
    float cos_phi = (ux1 * ux2) + (uy1 * uy2) + (uz1 * uz2);
    float epsilon = 0.00001f;

    // clamp the cosine against floating point error
    if (cos_phi > 1.0f) {
        cos_phi = 1.0f;
    }
    else if (cos_phi < -1.0f) {
        cos_phi = -1.0f;
    }

    float v_junction = 0.0f;

    // check edge cases
    if (cos_phi + epsilon > 1.0f) {
        // if the angle is 0 degrees, lines are in the same direction
        v_junction = buffer->arr[prev_idx].v_cruise;
    }
    else if (cos_phi - epsilon < -1.0f) {
        // if the angle is 180 degrees, lines are opposite directions
        v_junction = 0.0f;
    }
    else {
        // check other cases: 0 < angle < 180
        
        // theta is the real angle between the two physical lines
        float cos_half_phi = sqrtf((1.0f + cos_phi) / 2.0f); // trigo identity

        float j = JUNCTION_DEVIATION;   // the distance between the real junction point and the theoretical arc center
        float a = buffer->arr[prev_idx].max_path_acceleration; // the max centripetal acceleration is the same max path acceleration
        
        v_junction = sqrtf( (j * a * cos_half_phi) / (1.0f - cos_half_phi) );

        // clamp the junction speed, mustn't be higher than the cruise speed
        if (v_junction > buffer->arr[prev_idx].v_cruise) {
            v_junction = buffer->arr[prev_idx].v_cruise;
        }
        if (v_junction > buffer->arr[curr_idx].v_cruise) {
            v_junction = buffer->arr[curr_idx].v_cruise;
        }
    }

    // update the v_entry and v_exit of the appropriate motion blocks
    buffer->arr[prev_idx].v_exit = v_junction;
    buffer->arr[curr_idx].v_entry = v_junction;
}


/*
    does a backward pass on the planned motion profiles in the ring buffer, in order to check if the printer can
    decelerate safely based on the given entry velocity, path length and exit velocity of each profile.
*/
void backward_pass(RingBuffer* buffer) {
    // early exit
    if (buffer->count < 2) {
        return;
    }

    // compute indexes of the current watched profile and it's previous
    uint8_t curr_idx = (buffer->head + BUFFER_SIZE - 1) % BUFFER_SIZE;  // start at (head - 1)
    //printf("\ncurrent index: %d\n", curr_idx);
    
    // backward pass
    while (curr_idx != buffer->tail) {
        uint8_t prev_idx = (curr_idx + BUFFER_SIZE - 1) % BUFFER_SIZE;
        //printf("previous index: %d\n", prev_idx);

        // extract required parameters
        float v_entry = buffer->arr[curr_idx].v_entry;
        float v_exit = buffer->arr[curr_idx].v_exit;
        float distance = buffer->arr[curr_idx].path_length_mm;
        float acceleration = buffer->arr[curr_idx].max_path_acceleration;

        // compute max theoretical v_enty in order to safely reach v_exit
        float v_entry_max = sqrtf(v_exit * v_exit + 2.0f * acceleration * distance); // kinematical equation: Vf^2 = Vi^2 + 2*a*s

        //printf("v_entry: %f\n", v_entry);
        //printf("v_exit: %f\n", v_exit);
        //printf("distance: %f\n", distance);
        //printf("acceleration: %f\n", acceleration);
        //printf("v_entry_max: %f\n", v_entry_max);

        // clamp v_entry and previous profile v_exit
        if (v_entry > v_entry_max) {
            buffer->arr[curr_idx].v_entry = v_entry_max;
            buffer->arr[prev_idx].v_exit  = v_entry_max;
        }

        //printf("new v_entry: %f\n", buffer->arr[curr_idx].v_entry);

        curr_idx = prev_idx;
    }
}

/*
    does a forward pass on the planned motion profiles in order to check if the printer can
    accelerate to the desired cruise velocity based on the entry speed, path length, and exit velocity of each profile.
*/
void forward_pass(RingBuffer* buffer) {
    // early exit
    if (buffer->count < 2) {
        return;
    }

    // compute indexes of the current watched profile and it's next
    uint8_t curr_idx = buffer->tail;
    uint8_t next_idx = (curr_idx + 1) % BUFFER_SIZE;

    // forward pass
    while (next_idx != buffer->head) {
        // extract required parameters
        float v_entry = buffer->arr[curr_idx].v_entry;
        float v_exit = buffer->arr[curr_idx].v_exit;
        float distance = buffer->arr[curr_idx].path_length_mm;
        float acceleration = buffer->arr[curr_idx].max_path_acceleration;

        // compute max theoretical v_enty in order to safely reach v_exit
        float v_exit_max = sqrtf(v_entry * v_entry + 2.0f * acceleration * distance); // kinematical equation: Vf^2 = Vi^2 + 2*a*s

        // clamp v_exit and next profile v_entry
        if (v_exit > v_exit_max) {
            buffer->arr[curr_idx].v_exit = v_exit_max;
            buffer->arr[next_idx].v_entry = v_exit_max;
        }

        curr_idx = next_idx;
        next_idx = (curr_idx + 1) % BUFFER_SIZE;
    }
}

/*
    after setting safe entry and exit velocities, we need to verify if the desired cruise speed is achievable,
    if not clamp it. 
*/
void recalculate_cruise_speed(RingBuffer* buffer) {
    // early exit
    if (buffer->count < 2) {
        return;
    }

    uint8_t curr_idx = buffer->tail;

    // iterate the motion profiles buffer
    while (curr_idx != buffer->head) {
        // extract required parameters
        float v_entry = buffer->arr[curr_idx].v_entry;
        float v_exit = buffer->arr[curr_idx].v_exit;
        float distance = buffer->arr[curr_idx].path_length_mm;
        float acceleration = buffer->arr[curr_idx].max_path_acceleration;
        float v_cruise = buffer->arr[curr_idx].v_cruise;

        // compute max theoretical v_cruise: Vc^2 = (Vi^2 + Vf^2 + 2*a*d) / 2
        float v_cruise_max = sqrtf((v_exit * v_exit + v_entry * v_entry + 2.0f * acceleration * distance) / 2.0f);

        // clamp v_cruise
        if (v_cruise > v_cruise_max) {
            buffer->arr[curr_idx].v_cruise = v_cruise_max;
        }

        curr_idx = (curr_idx + 1) % BUFFER_SIZE;
    }
}


/*
    compute required parameters for the Step Generator
*/
void finalize_motion_profiles(RingBuffer* buffer) {
    // early exit
    if (buffer->count < 2) {
        return;
    }

    uint8_t curr_idx = buffer->tail;
    while (curr_idx != buffer->head) {

        // compute the electrical steps required in the acceleration, deceleration and cruise phase.
        float v_entry = buffer->arr[curr_idx].v_entry;
        float v_cruise = buffer->arr[curr_idx].v_cruise;
        float v_exit = buffer->arr[curr_idx].v_exit;
        float a = buffer->arr[curr_idx].max_path_acceleration;

        float accel_dist = (v_cruise * v_cruise - v_entry * v_entry) / (2.0f * a);
        float decel_dist = (v_cruise * v_cruise - v_exit * v_exit) / (2.0f * a);
        float cruise_dist = buffer->arr[curr_idx].path_length_mm - decel_dist - accel_dist;

        if (cruise_dist < 0.0001f) {
            cruise_dist = 0.0f;
        }
        
        buffer->arr[curr_idx].accel_steps = (uint32_t)lroundf(accel_dist * buffer->arr[curr_idx].master_steps_per_mm);
        buffer->arr[curr_idx].decel_steps = (uint32_t)lroundf(decel_dist * buffer->arr[curr_idx].master_steps_per_mm);
        buffer->arr[curr_idx].cruise_steps = (uint32_t)lroundf(cruise_dist * buffer->arr[curr_idx].master_steps_per_mm);
        
        curr_idx = (curr_idx + 1) % BUFFER_SIZE;
    }
}



/*
    dispatch a motion profile to the motion segments queue.
    dispatch the metadata to the PID controller or UI Queue.
*/
//void dispatch_motion() {
//
//}



/* 
    ************************************************
    auxiliary functions of create_initial_profile():
    ************************************************
*/


/*
    scale the cruise speed so each component of the vector doesnt exceed each axis top speed
*/
float limit_velocity(float v_target, float ux, float uy, float uz) {
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

    return v_allowed;
}

void compute_unit_vectors(PlannedMotion* motion, float deltas_mm[]) {
    float dx_mm = deltas_mm[0];
    float dy_mm = deltas_mm[1];
    float dz_mm = deltas_mm[2];
    float de_mm = deltas_mm[3];
    float total_length = motion->total_vector_length;

    float ux = 0.0f;
    float uy = 0.0f;
    float uz = 0.0f;
    float ue = 0.0f;
    
    // compute unit vectors of the 4d vector
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

    // calculate cartesian unit vectors
    ux = 0;
    uy = 0;
    uz = 0;
    float cartesian_length = motion->path_length_mm;

    if (cartesian_length > 0.0001f) {
        float len_inv = 1.0f / cartesian_length;  // calculate divison once
        ux = dx_mm * len_inv;
        uy = dy_mm * len_inv;
        uz = dz_mm * len_inv;
    }

    motion->cartesian_unit_vec[0] = ux;
    motion->cartesian_unit_vec[1] = uy;
    motion->cartesian_unit_vec[2] = uz;
}

void compute_max_path_and_vector_acceleration(PlannedMotion* motion) {
    float accelerations[NUM_AXES] = {0, 0, 0, 0};
    float ux = motion->unit_vec[0];
    float uy = motion->unit_vec[1];
    float uz = motion->unit_vec[2];
    float ue = motion->unit_vec[3];

    // compute max vector acceleration
    if (fabsf(ux) > 0.0001f) accelerations[0] = MAX_ACCELERATION_X / fabsf(ux);
    if (fabsf(uy) > 0.0001f) accelerations[1] = MAX_ACCELERATION_Y / fabsf(uy);
    if (fabsf(uz) > 0.0001f) accelerations[2] = MAX_ACCELERATION_Z / fabsf(uz);
    if (fabsf(ue) > 0.0001f) accelerations[3] = MAX_ACCELERATION_E / fabsf(ue);

    motion->max_vector_acceleration = accelerations[0];
    for (uint32_t i = 1; i < NUM_AXES; i++) {
        if (accelerations[i] < motion->max_vector_acceleration) {
            motion->max_vector_acceleration = accelerations[i];
        }
    }

    // compute max path acceleration
    float path_accels[NUM_AXES - 1] = {1e9f, 1e9f, 1e9f};
    ux = motion->cartesian_unit_vec[0];
    uy = motion->cartesian_unit_vec[1];
    uz = motion->cartesian_unit_vec[2];

    if (fabsf(ux) > 0.0001f) path_accels[0] = MAX_ACCELERATION_X / fabsf(ux);
    if (fabsf(uy) > 0.0001f) path_accels[1] = MAX_ACCELERATION_Y / fabsf(uy);
    if (fabsf(uz) > 0.0001f) path_accels[2] = MAX_ACCELERATION_Z / fabsf(uz);

    motion->max_path_acceleration = path_accels[0];
    for (uint32_t i = 1; i < NUM_AXES - 1; i++) {
        if (path_accels[i] < motion->max_path_acceleration) {
            motion->max_path_acceleration = path_accels[i];
        }
    }
}

/*
    computes initial trapezoidal profile velocities: v_start, v_end and v_cruise
*/
void compute_profile_velocities(GCodeCommand* gcode_cmd, PlannedMotion* motion, float* active_feedrate) {
    float ux = motion->cartesian_unit_vec[0];
    float uy = motion->cartesian_unit_vec[1];
    float uz = motion->cartesian_unit_vec[2];



    if (gcode_cmd->has_F) *active_feedrate = (gcode_cmd->F) / 60.0f;       // extract desired feedrate and convert to mm/s
    motion->v_cruise = limit_velocity(*active_feedrate, ux, uy, uz);    // find max feasible cruise speed
    motion->v_entry = motion->v_exit = 0.0f;                            // set default enter and exit speeds
}

void update_target_coordinate(GCodeCommand* gcode_cmd, PointMM* target_mm, bool absolute_mode) {
    if (gcode_cmd->has_X) target_mm->x = (absolute_mode) ? gcode_cmd->X : target_mm->x + gcode_cmd->X;
    if (gcode_cmd->has_Y) target_mm->y = (absolute_mode) ? gcode_cmd->Y : target_mm->y + gcode_cmd->Y;
    if (gcode_cmd->has_Z) target_mm->z = (absolute_mode) ? gcode_cmd->Z : target_mm->z + gcode_cmd->Z;
    if (gcode_cmd->has_E) target_mm->e = (absolute_mode) ? gcode_cmd->E : target_mm->e + gcode_cmd->E;
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
void compute_path_and_vector_lengths(PlannedMotion* motion, float deltas_mm[]) {
    float cartesian_length = compute_cartesian_length(deltas_mm);
    float total_length = cartesian_length;
    float de_mm = deltas_mm[3];

    // if move is pure extrusion, use E length
    if (cartesian_length < 0.0001f) {
        total_length = fabsf(de_mm);
    }

    motion->path_length_mm = cartesian_length;
    motion->total_vector_length = total_length;
}

/*
    finds the master axis and computes its electrical steps
*/
void compute_master_axis_steps(PlannedMotion* motion) {
    uint32_t master_steps = 0;
    uint8_t master_axis = 0;    // x - 0, y - 1, z - 2, e - 3; 
    
    for (uint32_t i = 0; i < NUM_AXES; i++) {
        uint32_t axis_steps = motion->steps[i];
        if (axis_steps > master_steps) {
            master_steps = axis_steps;
            master_axis = i;
        }
    }

    motion->master_axis = master_axis;
    motion->master_steps = master_steps;
}

/*
    computes the velocity scale factor based on master axis type
*/
void compute_master_axis_steps_per_mm(PlannedMotion* motion) {
    // steps per mm for each axis: x, y, z, e
    float steps_per_mm[] = {STEPS_PER_MM_BELT, STEPS_PER_MM_BELT, STEPS_PER_MM_SCREW, STEPS_PER_MM_GEAR};
    motion->master_steps_per_mm = steps_per_mm[motion->master_axis];
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

void pop(RingBuffer* buffer) {
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