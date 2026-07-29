#include "config.h"
#include "motion_planner.h"
#include "gcode_parser.h"

#include <math.h>

void plan_motion(GCodeCommand* gcode_cmd, PlannedMotion* motion) {

    static float v_allowed = 0.0f;
    static Point current_position = {0.0f, 0.0f, 0.0f, 0.0f};
    Point target;
    
    // extract target coordinates
    if (gcode_cmd->has_X) target.x = gcode_cmd->X;
    if (gcode_cmd->has_Y) target.y = gcode_cmd->Y;
    if (gcode_cmd->has_Z) target.z = gcode_cmd->Z;
    if (gcode_cmd->has_E) target.e = gcode_cmd->E;

    // compute deltas
    float dx = target.x - current_position.x;
    float dy = target.y - current_position.y;
    float dz = target.z - current_position.z;
    float de = target.e - current_position.e;

    // infer axes motion directions
    if (dx < 0.0f) motion->dir_bits |= (1 << 0);
    if (dy < 0.0f) motion->dir_bits |= (1 << 1);   
    if (dz < 0.0f) motion->dir_bits |= (1 << 2);   
    if (de < 0.0f) motion->dir_bits |= (1 << 3);   

    current_position = target;  // update the current position for the future command

    // compute total vector length and unit vectors
    float total_length = calc_length(dx, dy, dz, de);
    float ux, uy, uz, ue = 0.0f;
    if (fabs(dx) > 0.0001f) ux = dx / total_length;  // can't compare a float to zero
    if (fabs(dy) > 0.0001f) uy = dy / total_length;
    if (fabs(dz) > 0.0001f) uz = dz / total_length;
    if (fabs(de) > 0.0001f) ue = de / total_length;

    // set top feasible nominal velocity
    if (gcode_cmd->has_F) {
        float v_target = (gcode_cmd->F) / 60.0f;                // extract the feedrate and convert to mm/s
        v_allowed = limit_velocity(v_target, ux, uy, uz, ue);   // find max feasible nominal cruise speed
    }
}

float limit_velocity(float v_target, float ux, float uy, float uz, float ue) {
    float v_allowed = v_target;     // v_target > 0, it is a scalar

    if (fabs(ux) > 0.0001f) {
        float vx_limit = MAX_VELOCITY_X / fabs(ux);     // compute max x axis velocity limit
        if (vx_limit < v_allowed) v_allowed = vx_limit; // clamp cruise speed
    }

    if (fabs(uy) > 0.0001f) {
        float vy_limit = MAX_VELOCITY_Y / fabs(uy);     
        if (vy_limit < v_allowed) v_allowed = vy_limit; 
    }

    if (fabs(uz) > 0.0001f) {
        float vz_limit = MAX_VELOCITY_Z / fabs(uz);     
        if (vz_limit < v_allowed) v_allowed = vz_limit; 
    }

    if (fabs(ue) > 0.0001f) {
        float ve_limit = MAX_VELOCITY_E / fabs(ue);     
        if (ve_limit < v_allowed) v_allowed = ve_limit; 
    }

    return v_allowed;
}

float max(float dx, float dy, float dz, float de) {
    float maximum = 0;

    if (fabs(dx) > maximum) maximum = dx;
    if (fabs(dy) > maximum) maximum = dy;
    if (fabs(dz) > maximum) maximum = dz;
    if (fabs(de) > maximum) maximum = de;

    return maximum;
}

float calc_length(float dx, float dy, float dz, float de) {
    return sqrtf(dx*dx + dy*dy+ dz*dz + de*de);
}








