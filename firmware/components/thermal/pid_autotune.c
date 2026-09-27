#include "pid_autotune.h"

#ifndef PI
#define PI 3.14159f
#endif

/* Minimum time between relay switches (as in Marlin). Sensor noise near the setpoint
   is larger than the hysteresis band and would otherwise flip the relay back within
   a sample or two, counting fake short cycles and corrupting the gains. */
#define AUTOTUNE_MIN_SWITCH_SEC 5.0f

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
    tuner->last_switch_time = 0.0f;
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

    /* 2. Track Crest (High) and Trough (Low) Peaks.
       Thermal lag puts the crest AFTER switch-off and the trough AFTER switch-on,
       so peak_high is re-armed at switch-off and peak_low at switch-on. */
    if (current_temp > tuner->peak_high) tuner->peak_high = current_temp;
    if (current_temp < tuner->peak_low)  tuner->peak_low  = current_temp;

    /* 3. Relay Switching Logic with Hysteresis Band and a minimum dwell per state */
    bool may_switch = (tuner->elapsed_time - tuner->last_switch_time) >= AUTOTUNE_MIN_SWITCH_SEC;

    if (tuner->heating && may_switch &&
        current_temp >= (tuner->config.target_temp + tuner->config.hysteresis)) {
        /* Temperature exceeded upper threshold: Switch Relay OFF */
        tuner->heating = false;
        tuner->last_switch_time = tuner->elapsed_time;
        
        if (tuner->state == AUTOTUNE_STATE_HEATING) {
            /* Ramp-up complete, entering steady oscillation cycles */
            tuner->state = AUTOTUNE_STATE_CYCLING;
            tuner->cycle_start_time = tuner->elapsed_time;
        } else {
            /* Completed a full oscillation period: peak_high holds the crest since the
               previous switch-off, peak_low the trough since the last switch-on. */
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

        /* Re-arm high peak tracker for the crest that follows this switch-off */
        tuner->peak_high = current_temp;

    } else if (!tuner->heating && may_switch &&
               current_temp <= (tuner->config.target_temp - tuner->config.hysteresis)) {
        /* Temperature dropped below lower threshold: Switch Relay ON */
        tuner->heating = true;
        tuner->last_switch_time = tuner->elapsed_time;
        /* Re-arm low peak tracker for the trough that follows this switch-on */
        tuner->peak_low = current_temp;
    }

    /* 4. Calculate Final Gains on Completion */
    if (tuner->measurement_count >= tuner->config.requested_cycles) {
        float avg_period = tuner->period_sum / (float)tuner->measurement_count;
        float avg_amplitude = tuner->amplitude_sum / (float)tuner->measurement_count;

        if (avg_amplitude > 0.001f && avg_period > 0.001f) {
            /* Ultimate Gain: Ku = 4d / (pi * a), with relay amplitude d = output_power / 2
               (output swings 0..P) and oscillation amplitude a = peak-to-peak / 2. */
            float ku = (4.0f * tuner->config.output_power) / (PI * avg_amplitude);

            if (tuner->config.method == TUNING_METHOD_TYREUS_LUYBEN) {
                /* Tyreus-Luyben Formula (Bed Focus: Low Overshoot) */
                tuner->calculated_gains.kp = 0.45f * ku;
                tuner->calculated_gains.ki = tuner->calculated_gains.kp / (2.2f * avg_period);
                tuner->calculated_gains.kd = (tuner->calculated_gains.kp * avg_period) / 6.3f;
            } else {
                /* Ziegler-Nichols Formula (Hotend Focus: Fast Response) */
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