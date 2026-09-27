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

/* Runaway protection has two explicit phases (as in Marlin):
   1. Heating up (target not yet reached): at >80% power the temperature must rise
      RUNAWAY_TEMP_THRESHOLD within the period, or the heater/sensor is not working.
   2. Holding (target reached once): fault if the temperature stays more than
      *_HOLD_BAND_C below target for *_HOLD_PERIOD_SEC. Rule 1 cannot be used here:
      after autotune switches the heater back on, thermal lag keeps the temperature
      falling for several seconds, so a healthy heater shows no net rise in 20 s. */
#define RUNAWAY_TEMP_THRESHOLD      2.0f
#define HOTEND_RUNAWAY_PERIOD_SEC   20.0f
#define BED_RUNAWAY_PERIOD_SEC      60.0f
#define HOTEND_HOLD_BAND_C          4.0f
#define HOTEND_HOLD_PERIOD_SEC      40.0f
#define BED_HOLD_BAND_C             5.0f
#define BED_HOLD_PERIOD_SEC         90.0f

/* M303 autotune: measured cycles (plus one discarded warm-up cycle) and a hard timeout.
   The bed heats and cools slowly, so it needs far longer than the hotend. */
#define HOTEND_AUTOTUNE_CYCLES      5
#define HOTEND_AUTOTUNE_TIMEOUT_SEC 1200.0f
#define BED_AUTOTUNE_CYCLES         4
#define BED_AUTOTUNE_TIMEOUT_SEC    3600.0f
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
    float             runaway_period_sec;
    bool              reached_target;  // selects the runaway phase: false = heating up, true = holding
    float             below_band_sec;  // holding phase: time spent below (target - hold_band)
    float             hold_band;
    float             hold_period_sec;
    int               autotune_cycles;
    float             autotune_timeout_sec;
    const char*       name;
} heater_channel_t;

/* Initialise PID gains/limits and per-heater safety limits. Call once, after nvs_flash_init().
   Gains come from flash when an autotune has saved them, otherwise the built-in defaults. */
void thermal_heaters_init(heater_channel_t heaters[]);

/* Persist autotuned gains to NVS flash; they are loaded on every boot from then on. */
bool thermal_save_gains(heater_source_t heater, const pid_gains_t *gains);

void thermal_process_command(heater_channel_t heaters[]);

/* Temperature the heater is currently driving toward: the PID setpoint, the autotune
   target while tuning, 0 when off or faulted. */
float thermal_active_target(const heater_channel_t *h);

/* Latch a thermal fault: all heaters are forced off until reboot. */
void thermal_set_fault(void);
bool thermal_fault_active(void);

/* Published by the thermal task each cycle, read by the planner. */
void thermal_publish_status(const heater_channel_t heaters[]);
uint32_t thermal_last_applied_seq(void);
bool thermal_at_target(heater_source_t heater);
bool thermal_autotune_active(heater_source_t heater);
float thermal_get_temp(heater_source_t heater);
float thermal_get_target(heater_source_t heater);
bool thermal_set_target_temp(float target_temp);
bool thermal_start_autotune(float target_temp, tuning_method_t method);
bool thermal_turn_off(void);
bool thermal_update_gains(float kp, float ki, float kd);

#endif // THERMAL_TASK_H