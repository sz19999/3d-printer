#ifndef ADC_H
#define ADC_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

/* Recommended GPIO Mapping for ESP32-S3 (ADC Unit 1) */
#define HOTEND_ADC_CHANNEL  ADC_CHANNEL_3  /* GPIO 4 */
#define BED_ADC_CHANNEL     ADC_CHANNEL_4  /* GPIO 5 */

typedef struct {
    adc_oneshot_unit_handle_t unit_handle;
    
    /* Hotend Calibration */
    adc_cali_handle_t hotend_cali_handle;
    bool hotend_cali_enabled;
    
    /* Bed Calibration */
    adc_cali_handle_t bed_cali_handle;
    bool bed_cali_enabled;
} dual_adc_t;

esp_err_t dual_adc_init(dual_adc_t *handle);
esp_err_t dual_adc_read_channel_mv(dual_adc_t *handle, adc_channel_t channel, uint32_t *out_mv);

#endif /* ADC_H */