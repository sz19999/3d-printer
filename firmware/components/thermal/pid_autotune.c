#include "pid_autotune.h"

#ifndef PI
#define PI 3.14159f
#endif

void autotune_init(autotune_t *tuner, const autotune_config_t *config) {
    if (!tuner || !config) return;

    tuner->config = *config;
    tuner->state = AUTOTUNE_STATE_HEATING;
    tuner->current_cycle = 0;
    tuner->heating = true;
    tuner->peak_high = config->target_temp;
    tuner->peak_low = config->target_temp;
    tuner->elapsed_time = 0.0f;
    tuner->cycle_start_time = 0.0f;
    tuner->period_sum = 0.0f;
    tuner->amplitude_sum = 0.0f;
    tuner->measurement_count = 0;
    tuner->calculated_gains.kp = 0.0f;
    tuner->calculated_gains.ki = 0.0f;
    tuner->calculated_gains.kd = 0.0f;
}

float autotune_step(autotune_t *tuner, float current_temp, float dt_seconds) {
    if (!tuner) return 0.0f;

    if (tuner->state == AUTOTUNE_STATE_COMPLETE || tuner->state == AUTOTUNE_STATE_FAILED) {
        return 0.0f; // Keep heater OFF in terminal states
    }

    tuner->elapsed_time += dt_seconds;

    /* 1. Timeout Guard */
    if (tuner->elapsed_time > tuner->config.timeout_seconds) {
        tuner->state = AUTOTUNE_STATE_FAILED;
        return 0.0f;
    }

    /* 2. Track Crest (High) and Trough (Low) Peaks */
    if (current_temp > tuner->peak_high) tuner->peak_high = current_temp;
    if (current_temp < tuner->peak_low)  tuner->peak_low  = current_temp;

    /* 3. Relay Switching Logic with Hysteresis Band */
    if (tuner->heating && current_temp >= (tuner->config.target_temp + tuner->config.hysteresis)) {
        /* Temperature exceeded upper threshold: Switch Relay OFF */
        tuner->heating = false;
        
        if (tuner->state == AUTOTUNE_STATE_HEATING) {
            /* Ramp-up complete, entering steady oscillation cycles */
            tuner->state = AUTOTUNE_STATE_CYCLING;
            tuner->cycle_start_time = tuner->elapsed_time;
        } else {
            /* Completed a full oscillation period */
            float period = tuner->elapsed_time - tuner->cycle_start_time;
            float amplitude = tuner->peak_high - tuner->peak_low;

            /* Ignore initial transient cycle for cleaner average math */
            if (tuner->current_cycle > 0) {
                tuner->period_sum += period;
                tuner->amplitude_sum += amplitude;
                tuner->measurement_count++;
            }

            tuner->cycle_start_time = tuner->elapsed_time;
            tuner->current_cycle++;
        }

        /* Reset low peak tracker for upcoming trough */
        tuner->peak_low = current_temp;

    } else if (!tuner->heating && current_temp <= (tuner->config.target_temp - tuner->config.hysteresis)) {
        /* Temperature dropped below lower threshold: Switch Relay ON */
        tuner->heating = true;
        /* Reset high peak tracker for upcoming crest */
        tuner->peak_high = current_temp;
    }

    /* 4. Calculate Final Gains on Completion */
    if (tuner->measurement_count >= tuner->config.requested_cycles) {
        float avg_period = tuner->period_sum / (float)tuner->measurement_count;
        float avg_amplitude = tuner->amplitude_sum / (float)tuner->measurement_count;

        if (avg_amplitude > 0.001f && avg_period > 0.001f) {
            /* Calculate Ultimate Gain (Ku) */
            float ku = (8.0f * tuner->config.output_power) / (PI * avg_amplitude);

            if (tuner->config.method == TUNING_METHOD_TYREUS_LUYBEN) {
                /* Tyreus-Luyben Formula (Hotend Focus: Zero Overshoot) */
                tuner->calculated_gains.kp = 0.45f * ku;
                tuner->calculated_gains.ki = tuner->calculated_gains.kp / (2.2f * avg_period);
                tuner->calculated_gains.kd = (tuner->calculated_gains.kp * avg_period) / 6.3f;
            } else {
                /* Ziegler-Nichols Formula (Bed Focus: Fast Heat-Up) */
                tuner->calculated_gains.kp = 0.60f * ku;
                tuner->calculated_gains.ki = (2.0f * tuner->calculated_gains.kp) / avg_period;
                tuner->calculated_gains.kd = (tuner->calculated_gains.kp * avg_period) / 8.0f;
            }

            tuner->state = AUTOTUNE_STATE_COMPLETE;
        } else {
            tuner->state = AUTOTUNE_STATE_FAILED;
        }

        return 0.0f; // Turn off heater when finished
    }

    /* 5. Return Binary Relay Output */
    return tuner->heating ? tuner->config.output_power : 0.0f;
}