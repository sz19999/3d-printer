#include "heaters_fans.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "MOSFET_DRIVER";

#define MAX_DUTY_VALUE ((1 << PWM_RES_BITS) - 1) // 1023

typedef struct {
    gpio_num_t gpio_num;
    ledc_channel_t channel;
    ledc_timer_t timer;
    const char *name;
} mosfet_config_t;

static const mosfet_config_t g_mosfet_map[PWM_CH_NUM] = {
    [PWM_CHANNEL_HOTEND] = { .gpio_num = HOTEND_GPIO,  .channel = LEDC_CHANNEL_0, .timer = LEDC_TIMER_0, .name = "Hotend"  },
    [PWM_CHANNEL_HEATBED] = { .gpio_num = HEATBED_GPIO, .channel = LEDC_CHANNEL_1, .timer = LEDC_TIMER_0, .name = "Heatbed" },
    [PWM_CHANNEL_FAN]     = { .gpio_num = FAN_GPIO,     .channel = LEDC_CHANNEL_2, .timer = LEDC_TIMER_1, .name = "Fan"     }
};

void mosfet_driver_init(void) {
    ESP_LOGI(TAG, "Initializing MOSFET PWM drivers...");

    // 1. Configure Timer 0 (Heaters: 100 Hz)
    ledc_timer_config_t heater_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = PWM_RESOLUTION,
        .freq_hz          = HEATER_PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    if (ledc_timer_config(&heater_timer) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Heater PWM Timer!");
        return;
    }

    // 2. Configure Timer 1 (Fan: 25 kHz)
    ledc_timer_config_t fan_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_1,
        .duty_resolution  = PWM_RESOLUTION,
        .freq_hz          = FAN_PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    if (ledc_timer_config(&fan_timer) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Fan PWM Timer!");
        return;
    }

    // 3. Configure LEDC Channels
    for (int i = 0; i < PWM_CH_NUM; i++) {
        ledc_channel_config_t ledc_channel = {
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = g_mosfet_map[i].channel,
            .timer_sel      = g_mosfet_map[i].timer,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = g_mosfet_map[i].gpio_num,
            .duty           = 0, // Ensure output starts OFF
            .hpoint         = 0
        };
        
        if (ledc_channel_config(&ledc_channel) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure LEDC channel for %s (GPIO %d)", 
                     g_mosfet_map[i].name, g_mosfet_map[i].gpio_num);
        } else {
            ESP_LOGI(TAG, "%s channel initialized on GPIO %d", 
                     g_mosfet_map[i].name, g_mosfet_map[i].gpio_num);
        }
    }
}

void mosfet_set_duty_raw(pwm_channel_t chan, uint32_t duty) {
    if (chan >= PWM_CH_NUM) {
        ESP_LOGE(TAG, "Invalid channel index: %d", chan);
        return;
    }

    if (duty > MAX_DUTY_VALUE) {
        ESP_LOGW(TAG, "%s duty raw value %lu exceeds max (%d). Clamping to max.", 
                 g_mosfet_map[chan].name, (unsigned long)duty, MAX_DUTY_VALUE);
        duty = MAX_DUTY_VALUE;
    }

    ledc_channel_t ledc_chan = g_mosfet_map[chan].channel;
    
    if (ledc_set_duty(LEDC_LOW_SPEED_MODE, ledc_chan, duty) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set duty for %s", g_mosfet_map[chan].name);
        return;
    }

    if (ledc_update_duty(LEDC_LOW_SPEED_MODE, ledc_chan) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update duty cycle for %s", g_mosfet_map[chan].name);
    }
}

void mosfet_set_duty_percent(pwm_channel_t chan, float percentage) {
    if (chan >= PWM_CH_NUM) {
        ESP_LOGE(TAG, "Invalid channel index: %d", chan);
        return;
    }

    if (percentage < 0.0f) {
        ESP_LOGW(TAG, "%s requested duty lower than 0.0%% (%.2f%%). Clamping to 0.0%%", 
                 g_mosfet_map[chan].name, percentage);
        percentage = 0.0f;
    } else if (percentage > 100.0f) {
        ESP_LOGW(TAG, "%s requested duty higher than 100.0%% (%.2f%%). Clamping to 100.0%%", 
                 g_mosfet_map[chan].name, percentage);
        percentage = 100.0f;
    }

    uint32_t duty_raw = (uint32_t)((percentage / 100.0f) * MAX_DUTY_VALUE);
    mosfet_set_duty_raw(chan, duty_raw);
}

void mosfet_emergency_shutdown(void) {
    ESP_LOGE(TAG, "!!! EMERGENCY SHUTDOWN TRIGGERED !!! Shutting off all MOSFET outputs.");
    for (int i = 0; i < PWM_CH_NUM; i++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, g_mosfet_map[i].channel, 0); // write to the registers
        ledc_update_duty(LEDC_LOW_SPEED_MODE, g_mosfet_map[i].channel); // then update the pwm channel
    }
}