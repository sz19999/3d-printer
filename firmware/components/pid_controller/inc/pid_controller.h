#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float kd;
} pid_gains_t;

typedef struct {
    float min_output;   // 0.0f (0% duty cycle)
    float max_output;   // 1023.0f (100% duty cycle for 10-bit PWM)
    float windup_guard; // temperature error threshold (°C) to enable Integral
} pid_limits_t;

typedef struct {
    pid_gains_t gains;
    pid_limits_t limits;
    float setpoint;

    float integral_accumulator;
    float last_process_variable;
    bool is_first_run;
} pid_controller_t;

void pid_init(pid_controller_t *pid, const pid_gains_t *gains, const pid_limits_t *limits);
void pid_set_target(pid_controller_t *pid, float setpoint);
void pid_reset(pid_controller_t* pid);
float pid_compute(pid_controller_t *pid, float process_variable, float dt_seconds);

#endif