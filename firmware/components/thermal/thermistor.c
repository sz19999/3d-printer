#include "thermistor.h"
#include "esp_log.h"

static const char *TAG = "THERMISTOR";

const thermistor_config_t THERMISTOR_NTC3950_DEFAULT = {
    .r_pullup = 4700.0f,    /* 4.7 kOhm pull-up resistor */
    .v_ref_mv = 3300.0f,    /* 3.3V rail */
    .r0       = 100000.0f,  /* 100k Thermistor */
    .beta     = 3950.0f,    /* NTC 3950 Beta value */
    .t0_k     = 298.15f     /* 25°C in Kelvin */
};

float thermistor_mv_to_celsius(uint32_t v_out_mv, const thermistor_config_t *config) {
    if (!config) return -273.15f;

    /* Guard against disconnect (voltage near 3.3V rail) or short circuit (voltage near 0V) */
    if (v_out_mv >= (uint32_t)config->v_ref_mv - 10) {
        ESP_LOGW(TAG, "Sensor disconnected / open circuit detected (%lu mV)", v_out_mv);
        return -273.15f;
    }
    if (v_out_mv <= 10) {
        ESP_LOGW(TAG, "Sensor short-circuit detected (%lu mV)", v_out_mv);
        return -273.15f;
    }

    float v_out = (float)v_out_mv;

    /* 1. Calculate Thermistor Resistance (R_th) */
    float r_th = config->r_pullup * (v_out / (config->v_ref_mv - v_out));

    /* 2. Beta Equation to calculate Temperature in Kelvin */
    float temp_k = 1.0f / ((1.0f / config->t0_k) + (1.0f / config->beta) * logf(r_th / config->r0));

    /* 3. Convert Kelvin to Celsius */
    return temp_k - 273.15f;
}