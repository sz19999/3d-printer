#include "thermal_task.h"
#include "heaters_fans.h"
#include "adc_filter.h"
#include "thermistor.h"
#include "adc.h"
#include "pid_controller.h"
#include "pid_autotune.h"
#include "motion_planner.h"
#include "esp_log.h"

extern const char *TAG_THERMAL;
extern QueueHandle_t thermal_cmds_queue;

void thermal_process_command(heater_channel_t heaters[]) {
    thermal_cmd_t cmd;

    /* Drain all pending commands in the queue non-blockingly */
    if (xQueueReceive(thermal_cmds_queue, &cmd, 0) == pdTRUE) {
        
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
            return; // Skip heater dispatch logic
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
            return;
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

