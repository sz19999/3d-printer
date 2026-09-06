#ifndef THERMAL_TASK_H
#define THERMAL_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "pid_autotune.h"
#include "pid_controller.h"
#include "adc_filter.h"
#include "heaters_fans.h"
#include "adc.h"
#include "thermistor.h"

#define RUNAWAY_TEMP_THRESHOLD  2.0f
#define RUNAWAY_TIME_LIMIT_SEC  20.0f
#define HOTEND_AUTO_FAN_THRESHOLD_TEMP 50.0f

typedef enum {
    THERMAL_STATE_OFF,
    THERMAL_STATE_PID_RUNNING,
    THERMAL_STATE_AUTOTUNE_RUNNING,
    THERMAL_STATE_FAULT
} thermal_state_t;

typedef enum {
    HEATER_HOTEND,
    HEATER_BED
} heater_source_t;

typedef struct {
    thermal_state_t   state;
    adc_filter_t      filter;
    pid_controller_t  pid;
    autotune_t        autotune;
    pwm_channel_t     pwm_chan;
    adc_channel_t     adc_chan;
    float             current_temp;
    float             runaway_start_temp;
    float             runaway_timer_sec;
    const char*       name;
} heater_channel_t;

void thermal_process_command(heater_channel_t heaters[]);
bool thermal_set_target_temp(float target_temp);
bool thermal_start_autotune(float target_temp, tuning_method_t method);
bool thermal_turn_off(void);
bool thermal_update_gains(float kp, float ki, float kd);

#endif // THERMAL_TASK_H