#ifndef THERMISTOR_H
#define THERMISTOR_H

#include <stdint.h>

typedef struct {
    float r_pullup;     /* Pull-up resistor value in Ohms (typically 4700.0f) */
    float v_ref_mv;     /* Circuit reference voltage in mV (typically 3300.0f) */
    float r0;           /* Thermistor resistance at 25C in Ohms (typically 100000.0f) */
    float beta;         /* Beta coefficient (typically 3950.0f or 4267.0f) */
    float t0_k;         /* 25C in Kelvin (298.15f) */
} thermistor_config_t;


float thermistor_mv_to_celsius(uint32_t v_out_mv, const thermistor_config_t *config);

#endif /* THERMISTOR_H */