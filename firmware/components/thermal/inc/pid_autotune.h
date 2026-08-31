#ifndef PID_AUTO_H
#define PID_AUTO_H

#include <stdbool.h>
#include "pid_controller.h"

typedef enum {
    TUNING_METHOD_TYREUS_LUYBEN, // Recommended for Hotends (Zero Overshoot)
    TUNING_METHOD_ZIEGLER_NICHOLS // Recommended for Heated Beds (Fast Heat-Up)
} tuning_method_t;

typedef enum {
    AUTOTUNE_STATE_IDLE,
    AUTOTUNE_STATE_HEATING,
    AUTOTUNE_STATE_CYCLING,
    AUTOTUNE_STATE_COMPLETE,
    AUTOTUNE_STATE_FAILED
} autotune_state_t;

typedef struct {
    float target_temp;      // Target setpoint (°C)
    float output_power;     // Maximum PWM power during test (e.g., 255.0f) 
    float hysteresis;       // Noise band around setpoint (e.g., 0.5f to 1.0f °C)
    int requested_cycles;   // Typically 4 to 8 cycles 
    float timeout_seconds;  // Max time allowed before declaring a fault 
    tuning_method_t method; // Tuning algorithm formula to apply 
} autotune_config_t;

typedef struct {
    autotune_config_t config;
    autotune_state_t state;

    // Cycle & Oscillation Tracking
    int current_cycle;
    bool heating;
    float peak_high;
    float peak_low;
    
    // Timing & Math Accumulators
    float elapsed_time;
    float cycle_start_time;
    float period_sum;
    float amplitude_sum;
    int measurement_count;

    // Output Results
    pid_gains_t calculated_gains;
} autotune_t;


void autotune_init(autotune_t *tuner, const autotune_config_t *config);
float autotune_step(autotune_t *tuner, float current_temp, float dt_seconds);
void calculate_gains(autotune_t *tuner, tuning_method_t method, float ku, float pu);

#endif