#include "adc.h"
#include "esp_log.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "DUAL_ADC";

#define THERMISTOR_ATTEN ADC_ATTEN_DB_12

static bool init_calibration(adc_channel_t channel, adc_cali_handle_t *out_handle) {
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = channel,
        .atten = THERMISTOR_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_config, out_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration enabled for ADC Channel %d", channel);
        return true;
    }
    
    ESP_LOGW(TAG, "Calibration failed for Channel %d (%s). Using raw estimates.", 
             channel, esp_err_to_name(ret));
    return false;
}

esp_err_t dual_adc_init(dual_adc_t *handle) {
    if (!handle) return ESP_ERR_INVALID_ARG;

    /* Step 1: Initialize ADC Unit 1 */
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t err = adc_oneshot_new_unit(&init_config, &handle->unit_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ADC1 unit: %s", esp_err_to_name(err));
        return err;
    }

    /* Step 2: Configure Both Channels */
    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = THERMISTOR_ATTEN,
    };

    err = adc_oneshot_config_channel(handle->unit_handle, HOTEND_ADC_CHANNEL, &chan_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config Hotend channel: %s", esp_err_to_name(err));
        return err;
    }

    err = adc_oneshot_config_channel(handle->unit_handle, BED_ADC_CHANNEL, &chan_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config Bed channel: %s", esp_err_to_name(err));
        return err;
    }

    /* Step 3: Enable Calibration for Each Channel */
    handle->hotend_cali_enabled = init_calibration(HOTEND_ADC_CHANNEL, &handle->hotend_cali_handle);
    handle->bed_cali_enabled    = init_calibration(BED_ADC_CHANNEL, &handle->bed_cali_handle);

    return ESP_OK;
}

esp_err_t dual_adc_read_channel_mv(dual_adc_t *handle, adc_channel_t channel, uint32_t *out_mv) {
    if (!handle || !out_mv) return ESP_ERR_INVALID_ARG;

    adc_cali_handle_t cali_handle = NULL;
    bool cali_enabled = false;

    /* Select calibration handle corresponding to requested channel */
    if (channel == HOTEND_ADC_CHANNEL) {
        cali_handle  = handle->hotend_cali_handle;
        cali_enabled = handle->hotend_cali_enabled;
    } else if (channel == BED_ADC_CHANNEL) {
        cali_handle  = handle->bed_cali_handle;
        cali_enabled = handle->bed_cali_enabled;
    } else {
        ESP_LOGE(TAG, "Unsupported ADC Channel: %d", channel);
        return ESP_ERR_INVALID_ARG;
    }

    /* Read raw ADC hardware sample */
    int raw = 0;
    esp_err_t err = adc_oneshot_read(handle->unit_handle, channel, &raw);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Raw read failed on Channel %d: %s", channel, esp_err_to_name(err));
        return err;
    }

    /* Convert to calibrated voltage (mV) */
    if (cali_enabled) {
        int voltage_mv = 0;
        err = adc_cali_raw_to_voltage(cali_handle, raw, &voltage_mv);
        if (err == ESP_OK) {
            *out_mv = (uint32_t)voltage_mv;
            return ESP_OK;
        }
    }

    /* Fallback estimation if eFuse curve fitting wasn't available */
    *out_mv = (uint32_t)((raw * 3300) / 4095);
    return ESP_OK;
}