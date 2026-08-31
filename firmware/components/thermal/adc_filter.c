#include "adc_filter.h"
#include <string.h>

/* Helper function: Fast insertion sort to find the median of small array buffers */
static uint16_t calculate_median(uint16_t *arr, uint8_t size) {
    uint16_t sorted[MEDIAN_WINDOW_SIZE];
    memcpy(sorted, arr, size * sizeof(uint16_t));

    for (uint8_t i = 1; i < size; i++) {
        uint16_t key = sorted[i];
        int8_t j = i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }
    return sorted[size / 2];
}

void adc_filter_init(adc_filter_t *filter, float alpha) {
    if (!filter) return;

    memset(filter->buffer, 0, sizeof(filter->buffer));
    filter->buffer_index = 0;
    filter->buffer_filled = false;

    /* Clamp alpha between 0.01 and 1.0 */
    if (alpha <= 0.0f) alpha = 0.01f;
    if (alpha > 1.0f)  alpha = 1.0f;

    filter->alpha = alpha;
    filter->filtered_val = 0.0f;
    filter->initialized = false;
}

float adc_filter_update(adc_filter_t *filter, uint16_t raw_sample) {
    if (!filter) return 0.0f;

    /* Step 1: Insert sample into ring buffer */
    filter->buffer[filter->buffer_index] = raw_sample;
    filter->buffer_index = (filter->buffer_index + 1) % MEDIAN_WINDOW_SIZE;
    
    if (filter->buffer_index == 0) {
        filter->buffer_filled = true;
    }

    uint8_t active_size = filter->buffer_filled ? MEDIAN_WINDOW_SIZE : filter->buffer_index;

    /* Step 2: Extract Median (Completely rejects EMI spikes/glitches) */
    uint16_t median_sample = calculate_median(filter->buffer, active_size);

    /* Step 3: Single-Pole Low-Pass Filter (EWMA) */
    if (!filter->initialized) {
        /* Cold start: Initialize state directly to avoid slow ramp-up lag on boot */
        filter->filtered_val = (float)median_sample;
        filter->initialized = true;
    } else {
        /* IIR Equation: Y[n] = alpha * X[n] + (1 - alpha) * Y[n-1] */
        filter->filtered_val = (filter->alpha * (float)median_sample) + 
                               ((1.0f - filter->alpha) * filter->filtered_val);
    }

    return filter->filtered_val;
}

void adc_filter_reset(adc_filter_t *filter) {
    if (!filter) return;
    filter->buffer_index = 0;
    filter->buffer_filled = false;
    filter->initialized = false;
}