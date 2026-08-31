#ifndef ADC_FILTER_H
#define ADC_FILTER_H

#include <stdint.h>
#include <stdbool.h>

#define MEDIAN_WINDOW_SIZE 5 // Must be an odd number (3, 5, or 7)

typedef struct {
    /* Ring buffer for median outlier rejection */
    uint16_t buffer[MEDIAN_WINDOW_SIZE];
    uint8_t buffer_index;
    bool buffer_filled;

    /* Single-Pole Low-Pass Filter State */
    float alpha;         /* Smoothing factor: 0.0 < alpha <= 1.0 (Lower = stronger filtering) */
    float filtered_val;  /* State variable storing current filtered result */
    bool initialized;
} adc_filter_t;


void adc_filter_init(adc_filter_t *filter, float alpha);
float adc_filter_update(adc_filter_t *filter, uint16_t raw_sample);
void adc_filter_reset(adc_filter_t *filter);

#endif /* ADC_FILTER_H */