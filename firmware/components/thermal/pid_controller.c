#include "pid_controller.h"
#include <math.h>
#include <stdbool.h>

static float clampf(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

void pid_init(pid_controller_t *pid, const pid_gains_t *gains, const pid_limits_t *limits) {
    if (!pid) return;

    if (gains) pid->gains = *gains;
    if (limits) pid->limits = *limits;
    
    pid->setpoint = 0.0f;
    pid_reset(pid);
}

void pid_set_target(pid_controller_t *pid, float setpoint) {
    if (!pid) return;

    if (pid->setpoint != setpoint) {
        pid->setpoint = setpoint;
        pid->integral_accumulator = 0.0f; /* Reset integral on setpoint shift */
    }
}

void pid_reset(pid_controller_t *pid) {
    if (!pid) return;

    pid->integral_accumulator = 0.0f;
    pid->last_process_variable = 0.0f;
    pid->is_first_run = true;
}

float pid_compute(pid_controller_t *pid, float process_variable, float dt_seconds) {
    if (!pid) return 0.0f;

    // Protect against divide-by-zero or non-positive time intervals
    if (dt_seconds <= 0.0001f) {
        return 0.0f;
    }

    float error = pid->setpoint - process_variable;

    // Proportional Term
    float p_term = pid->gains.kp * error;

    // Integral Term with Anti-Windup Guard
    if (fabsf(error) <= pid->limits.windup_guard) {
        pid->integral_accumulator += error * dt_seconds;

        // Clamp the accumulator output component to prevent windup saturation
        if (pid->gains.ki > 0.0f) {
            float max_i = pid->limits.max_output / pid->gains.ki;
            pid->integral_accumulator = clampf(pid->integral_accumulator, -max_i, max_i);
        }
    } else {
        pid->integral_accumulator = 0.0f; // Clear accumulation outside window
    }
    float i_term = pid->gains.ki * pid->integral_accumulator;

    // Derivative Term (Derivative-on-Measurement)
    float d_term = 0.0f;
    if (!pid->is_first_run) {
        float rate_of_change = (process_variable - pid->last_process_variable) / dt_seconds;
        d_term = -pid->gains.kd * rate_of_change;
    } else {
        pid->is_first_run = false;
    }
    pid->last_process_variable = process_variable;

    // Output Summation & Saturation
    float raw_output = p_term + i_term + d_term;
    return clampf(raw_output, pid->limits.min_output, pid->limits.max_output);
}