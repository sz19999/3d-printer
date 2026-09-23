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

/* Temperature limits, sized for PLA only.
   *_MAX_TARGET: highest setpoint accepted; higher requests are clamped down.
   *_MAXTEMP:    hard limit; reading above it faults ALL heaters immediately.
   THERMAL_MINTEMP: a reading below it means a shorted/broken sensor -> fault. */
#define HOTEND_MAX_TARGET       230.0f
#define HOTEND_MAXTEMP          250.0f
#define BED_MAX_TARGET          70.0f
#define BED_MAXTEMP             85.0f
#define THERMAL_MINTEMP         5.0f

/* M109 / M190 "reached target" rule: within +/-window for TEMP_RESIDENCY_SEC. */
#define TEMP_WINDOW_HOTEND      3.0f
#define TEMP_WINDOW_BED         2.0f
#define TEMP_RESIDENCY_SEC      5.0f

#define THERMAL_CMD_QUEUE_LEN   8

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
    float             residency_sec;   // time spent inside the target window
    float             max_target;      // setpoint clamp
    float             max_temp;        // hard fault limit
    float             target_window;   // +/- band for "target reached"
    const char*       name;
} heater_channel_t;

/* Initialise PID gains/limits and per-heater safety limits. Call once. */
void thermal_heaters_init(heater_channel_t heaters[]);

void thermal_process_command(heater_channel_t heaters[]);

/* Latch a thermal fault: all heaters are forced off until reboot. */
void thermal_set_fault(void);
bool thermal_fault_active(void);

/* Published by the thermal task each cycle, read by the planner. */
void thermal_publish_status(const heater_channel_t heaters[]);
uint32_t thermal_last_applied_seq(void);
bool thermal_at_target(heater_source_t heater);
float thermal_get_temp(heater_source_t heater);
float thermal_get_target(heater_source_t heater);
bool thermal_set_target_temp(float target_temp);
bool thermal_start_autotune(float target_temp, tuning_method_t method);
bool thermal_turn_off(void);
bool thermal_update_gains(float kp, float ki, float kd);

#endif // THERMAL_TASK_H