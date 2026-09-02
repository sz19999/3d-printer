#ifndef HEATERS_FANS_H
#define HEATERS_FANS_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

#define HOTEND_GPIO         GPIO_NUM_7
#define HEATBED_GPIO        GPIO_NUM_6
#define PART_FAN_GPIO       GPIO_NUM_15

#define HEATER_PWM_FREQ_HZ  100                 // Low frequency for thermal stability & low EMI
#define FAN_PWM_FREQ_HZ     25000               // High frequency (25 kHz) to avoid audible motor hum
#define PWM_RESOLUTION      LEDC_TIMER_10_BIT   // 0 to 1023 range
#define PWM_RES_BITS        10
#define PWM_CH_NUM          3

typedef enum {
    PWM_CHANNEL_HOTEND = 0,
    PWM_CHANNEL_HEATBED,
    PWM_CHANNEL_PART_FAN
} pwm_channel_t;

/*
    Configures LEDC timers and channels for hotend, heatbed, and fan MOSFETs.
*/
void mosfet_driver_init(void);

/*
    Sets duty cycle using raw ticks (0 to 1023).
*/
void mosfet_set_duty_raw(pwm_channel_t chan, uint32_t duty);

/*
    Sets duty cycle as a percentage (0.0% to 100.0%).
*/
void mosfet_set_duty_percent(pwm_channel_t chan, float percentage);

/*
    Emergency shutdown: Immediately cuts duty cycle to 0 across all channels.
*/
void mosfet_emergency_shutdown(void);

#endif