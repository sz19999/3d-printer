#include "thermal_task.h"
#include "heaters_fans.h"
#include "adc_filter.h"
#include "thermistor.h"
#include "adc.h"
#include "pid_controller.h"
#include "pid_autotune.h"
#include "motion_planner.h"
#include "esp_log.h"

#define RUNAWAY_TEMP_THRESHOLD  2.0f
#define RUNAWAY_TIME_LIMIT_SEC  20.0f
#define HOTEND_AUTO_FAN_THRESHOLD_TEMP 50.0f

extern TaskHandle_t  xThermalTaskHandle;
extern QueueHandle_t thermal_cmds_queue;

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


const char *TAG_THERMAL = "THERMAL_TASK";


static void process_command(heater_channel_t heaters[]) {
    thermal_cmd_t cmd;

    /* Drain all pending commands in the queue non-blockingly */
    while (xQueueReceive(thermal_cmds_queue, &cmd, 0) == pdTRUE) {
        
        /* 1. Handle Fan Commands (M106 / M107) */
        if (cmd.cmd_num == 106 || cmd.cmd_num == 107) {
            float pwm_duty = 0.0f;
            
            if (cmd.cmd_num == 106) {
                // Scale 0-255 G-code S parameter to 0-1023 raw PWM duty (or direct 0-1023 depending on parser)
                pwm_duty = (cmd.fan_speed > 255.0f) ? cmd.fan_speed : (cmd.fan_speed * (1023.0f / 255.0f));
            } else {
                pwm_duty = 0.0f; // M107 Fan Off
            }

            mosfet_set_duty_raw(pwm_duty, PWM_CHANNEL_PART_FAN);
            ESP_LOGI(TAG_THERMAL, "[PART_FAN] Duty set to %.1f", pwm_duty);
            continue; // Skip heater dispatch logic
        }

        /* 2. Determine targeted heater channel */
        int idx = HEATER_HOTEND;
        if (cmd.cmd_num == 140 || cmd.cmd_num == 190) {
            idx = HEATER_BED;
        } else if (cmd.cmd_num == 303) {
            // Check M303 E parameter payload (0 = Hotend, 1 or -1 = Bed)
            // idx = (cmd.autotune_e_param < 0 || cmd.autotune_e_param == 1) ? HEATER_BED : HEATER_HOTEND;
        }

        heater_channel_t *h = &heaters[idx];

        /* Reject commands if channel is in thermal FAULT */
        if (h->state == THERMAL_STATE_FAULT && cmd.temp_target > 0.0f) {
            ESP_LOGW(TAG_THERMAL, "[%s] Command rejected: Channel in FAULT!", h->name);
            continue;
        }

        /* 3. Dispatch Heater Commands */
        switch (cmd.cmd_num) {
            case 104:
            case 109:   /* Hotend set target temp */
            case 140:
            case 190:   /* Bed set target temp */
                h->pid.setpoint = cmd.temp_target;
                adc_filter_reset(&h->filter);
                h->runaway_timer_sec = 0.0f;

                if (cmd.temp_target > 0.0f) {
                    h->state = THERMAL_STATE_PID_RUNNING;
                    ESP_LOGI(TAG_THERMAL, "[%s] Target temp updated to %.1f C", h->name, cmd.temp_target);
                } else {
                    h->state = THERMAL_STATE_OFF;
                    ESP_LOGI(TAG_THERMAL, "[%s] Powered down (target 0 C)", h->name);
                }
                break;

            case 303: { /* PID Autotune */
                autotune_config_t config = {
                    .target_temp = cmd.temp_target,
                    .output_power = 1023.0f,
                    .hysteresis = 0.5f,
                    .requested_cycles = 5,
                    .timeout_seconds = 300.0f,
                    .method = (idx == HEATER_BED) ? TUNING_METHOD_TYREUS_LUYBEN : TUNING_METHOD_ZIEGLER_NICHOLS
                };
                
                adc_filter_reset(&h->filter);
                autotune_init(&h->autotune, &config);
                h->runaway_timer_sec = 0.0f;
                h->state = THERMAL_STATE_AUTOTUNE_RUNNING;
                ESP_LOGI(TAG_THERMAL, "[%s] Autotune started at %.1f C", h->name, cmd.temp_target);
                break;
            }

            default:
                ESP_LOGW(TAG_THERMAL, "Unknown thermal command type: %d", cmd.cmd_num);
                break;
        }
    }
}

void thermal_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(100);
    const float dt_seconds = 0.100f;

    heater_channel_t heaters[2] = {
        [0] = { .name = "HOTEND", .pwm_chan = PWM_CHANNEL_HOTEND,  .adc_chan = HOTEND_ADC_CHANNEL },
        [1] = { .name = "BED",    .pwm_chan = PWM_CHANNEL_HEATBED, .adc_chan = BED_ADC_CHANNEL }
    };
    dual_adc_t sensors_adc;
    extern thermistor_config_t THERMISTOR_NTC3950_DEFAULT;

    adc_filter_init(&heaters[HEATER_HOTEND].filter, 0.15f);
    adc_filter_init(&heaters[HEATER_BED].filter, 0.1f);
    dual_adc_init(&sensors_adc);
    
    ESP_LOGI(TAG_THERMAL, "Thermal Control Task online (Core %d)", xPortGetCoreID());

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        /* Step 1: Command Processing (Heater & Fan Queue) */
        process_command(heaters);

        /* Step 3: Process control and safety loop for each channel independently */
        for (int i = 0; i < 2; i++) {
            heater_channel_t *h = &heaters[i];

            /* Read Sensor Signal */
            uint32_t raw_mv = 0;
            esp_err_t err = dual_adc_read_channel_mv(&sensors_adc, h->adc_chan, &raw_mv);
            if (err != ESP_OK) {
                ESP_LOGE(TAG_THERMAL, "[%s] ADC Read Failure! Triggering FAULT.", h->name);
                h->state = THERMAL_STATE_FAULT;
            }

            float smooth_mv = adc_filter_update(&h->filter, (float)raw_mv);
            h->current_temp = thermistor_mv_to_celsius(smooth_mv, &THERMISTOR_NTC3950_DEFAULT);

            float pwm_output = 0.0f;

            /* Execute Control Logic */
            switch (h->state) {
                case THERMAL_STATE_PID_RUNNING:
                    pwm_output = pid_compute(&h->pid, h->current_temp, dt_seconds);
                    
                    if (h->current_temp > (h->pid.setpoint + 20.0f)) {
                        ESP_LOGE(TAG_THERMAL, "[%s] FAULT: Temperature overshot target by >20C!", h->name);
                        h->state = THERMAL_STATE_FAULT;
                    }
                    break;

                case THERMAL_STATE_AUTOTUNE_RUNNING:
                    pwm_output = autotune_step(&h->autotune, h->current_temp, dt_seconds);

                    if (h->autotune.state == AUTOTUNE_STATE_COMPLETE) {
                        ESP_LOGI(TAG_THERMAL, "[%s] Autotune complete. Kp: %.2f, Ki: %.2f, Kd: %.2f",
                                 h->name,
                                 h->autotune.calculated_gains.kp,
                                 h->autotune.calculated_gains.ki,
                                 h->autotune.calculated_gains.kd);

                        pid_init(&h->pid, &h->autotune.calculated_gains, &h->pid.limits);
                        h->state = THERMAL_STATE_PID_RUNNING;

                    } else if (h->autotune.state == AUTOTUNE_STATE_FAILED) {
                        ESP_LOGE(TAG_THERMAL, "[%s] Autotune failed or timed out!", h->name);
                        h->state = THERMAL_STATE_FAULT;
                    }
                    break;

                case THERMAL_STATE_OFF:
                case THERMAL_STATE_FAULT:
                default:
                    pwm_output = 0.0f;
                    break;
            }

            /* Thermal Runaway Protection Logic */
            if (pwm_output > 818.0f && h->state != THERMAL_STATE_FAULT) {
                if (h->runaway_timer_sec == 0.0f) {
                    h->runaway_start_temp = h->current_temp;
                }
                
                h->runaway_timer_sec += dt_seconds;
                
                if (h->runaway_timer_sec >= RUNAWAY_TIME_LIMIT_SEC) {
                    if ((h->current_temp - h->runaway_start_temp) < RUNAWAY_TEMP_THRESHOLD) {
                        ESP_LOGE(TAG_THERMAL, "[%s] THERMAL RUNAWAY: Power applied but temp not rising!", h->name);
                        h->state = THERMAL_STATE_FAULT;
                    } else {
                        h->runaway_start_temp = h->current_temp;
                        h->runaway_timer_sec = 0.0f;
                    }
                }
            } else {
                h->runaway_start_temp = h->current_temp;
                h->runaway_timer_sec = 0.0f;
            }

            /* Drive Hardware PWM Output */
            if (h->state == THERMAL_STATE_FAULT) {
                mosfet_set_duty_raw(0.0f, h->pwm_chan);
            } else {
                mosfet_set_duty_raw(pwm_output, h->pwm_chan);
            }
        }
    }
}