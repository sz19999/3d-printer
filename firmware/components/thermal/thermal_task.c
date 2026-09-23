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

/* ---- Status shared with the planner task ----
   Written only by the thermal task, read by the planner. 32-bit aligned
   loads/stores are atomic on the ESP32-S3, so volatile is sufficient. */
static volatile bool     s_fault = false;
static volatile uint32_t s_last_applied_seq = 0;
static volatile float    s_temp[2]      = {0};
static volatile float    s_target[2]    = {0};
static volatile bool     s_at_target[2] = {false, false};

/* Conservative starting gains for a 10-bit output (0..1023) at a 100 ms loop.
   Not tuned for this hardware; they favour a slow, low-overshoot approach. */
static const pid_gains_t  HOTEND_DEFAULT_GAINS  = { .kp = 60.0f, .ki = 2.0f, .kd = 250.0f };
static const pid_limits_t HOTEND_DEFAULT_LIMITS = { .min_output = 0.0f, .max_output = 1023.0f, .windup_guard = 15.0f };
static const pid_gains_t  BED_DEFAULT_GAINS     = { .kp = 50.0f, .ki = 1.0f, .kd = 0.0f };
static const pid_limits_t BED_DEFAULT_LIMITS    = { .min_output = 0.0f, .max_output = 1023.0f, .windup_guard = 10.0f };

void thermal_heaters_init(heater_channel_t heaters[]) {
    heater_channel_t *hot = &heaters[HEATER_HOTEND];
    pid_init(&hot->pid, &HOTEND_DEFAULT_GAINS, &HOTEND_DEFAULT_LIMITS);
    hot->max_target    = HOTEND_MAX_TARGET;
    hot->max_temp      = HOTEND_MAXTEMP;
    hot->target_window = TEMP_WINDOW_HOTEND;

    heater_channel_t *bed = &heaters[HEATER_BED];
    pid_init(&bed->pid, &BED_DEFAULT_GAINS, &BED_DEFAULT_LIMITS);
    bed->max_target    = BED_MAX_TARGET;
    bed->max_temp      = BED_MAXTEMP;
    bed->target_window = TEMP_WINDOW_BED;

    for (int i = 0; i < 2; i++) {
        heaters[i].state = THERMAL_STATE_OFF;
        heaters[i].runaway_timer_sec = 0.0f;
        heaters[i].residency_sec = 0.0f;
    }
}

void thermal_set_fault(void) {
    if (!s_fault) {
        ESP_LOGE(TAG_THERMAL, "!!! THERMAL FAULT LATCHED - all heaters OFF until reboot !!!");
    }
    s_fault = true;
    mosfet_emergency_shutdown();
}

bool thermal_fault_active(void) { return s_fault; }
uint32_t thermal_last_applied_seq(void) { return s_last_applied_seq; }
bool thermal_at_target(heater_source_t heater) { return s_at_target[heater]; }
float thermal_get_temp(heater_source_t heater) { return s_temp[heater]; }
float thermal_get_target(heater_source_t heater) { return s_target[heater]; }

void thermal_publish_status(const heater_channel_t heaters[]) {
    for (int i = 0; i < 2; i++) {
        s_temp[i] = heaters[i].current_temp;
        s_target[i] = (heaters[i].state == THERMAL_STATE_OFF ||
                       heaters[i].state == THERMAL_STATE_FAULT) ? 0.0f : heaters[i].pid.setpoint;
        s_at_target[i] = (heaters[i].state == THERMAL_STATE_PID_RUNNING) &&
                         (heaters[i].residency_sec >= TEMP_RESIDENCY_SEC);
    }
}

static void apply_one_command(heater_channel_t heaters[], const thermal_cmd_t *cmd) {
    /* 1. Fan commands (M106 / M107) */
    if (cmd->cmd_num == 106 || cmd->cmd_num == 107) {
        float pwm_duty = 0.0f;
        if (cmd->cmd_num == 106) {
            // Scale 0-255 G-code S parameter to 0-1023 raw PWM duty
            pwm_duty = (cmd->fan_speed > 255.0f) ? cmd->fan_speed : (cmd->fan_speed * (1023.0f / 255.0f));
        }
        mosfet_set_duty_raw(PWM_CHANNEL_PART_FAN, (uint32_t)pwm_duty);
        ESP_LOGI(TAG_THERMAL, "[PART_FAN] Duty set to %.1f", pwm_duty);
        return;
    }

    /* 2. Determine targeted heater channel */
    int idx = HEATER_HOTEND;
    if (cmd->cmd_num == 140 || cmd->cmd_num == 190) {
        idx = HEATER_BED;
    }
    heater_channel_t *h = &heaters[idx];

    /* Reject heating requests while a fault is latched */
    if ((s_fault || h->state == THERMAL_STATE_FAULT) && cmd->temp_target > 0) {
        ESP_LOGW(TAG_THERMAL, "[%s] Command rejected: thermal FAULT is latched!", h->name);
        return;
    }

    /* 3. Dispatch heater commands */
    switch (cmd->cmd_num) {
        case 104:
        case 109:   /* Hotend set target temp */
        case 140:
        case 190: { /* Bed set target temp */
            float target = (float)cmd->temp_target;

            // Explicit PLA limit: clamp anything above the allowed setpoint.
            if (target > h->max_target) {
                ESP_LOGW(TAG_THERMAL, "[%s] Target %.0f C above limit, clamped to %.0f C",
                         h->name, target, h->max_target);
                target = h->max_target;
            }

            h->residency_sec = 0.0f;       // "reached" must be re-earned for the new target
            h->pid.setpoint = target;
            pid_reset(&h->pid);
            adc_filter_reset(&h->filter);
            h->runaway_timer_sec = 0.0f;

            if (target > 0.0f) {
                h->state = THERMAL_STATE_PID_RUNNING;
                ESP_LOGI(TAG_THERMAL, "[%s] Target temp updated to %.1f C", h->name, target);
            } else {
                h->state = THERMAL_STATE_OFF;
                ESP_LOGI(TAG_THERMAL, "[%s] Powered down (target 0 C)", h->name);
            }
            break;
        }

        case 303: { /* PID Autotune (not yet forwarded by the planner) */
            autotune_config_t config = {
                .target_temp = cmd->temp_target,
                .output_power = 1023.0f,
                .hysteresis = 0.5f,
                .requested_cycles = 5,
                .timeout_seconds = 300.0f,
                .method = (idx == HEATER_BED) ? TUNING_METHOD_TYREUS_LUYBEN : TUNING_METHOD_ZIEGLER_NICHOLS
            };

            h->residency_sec = 0.0f;
            adc_filter_reset(&h->filter);
            autotune_init(&h->autotune, &config);
            h->runaway_timer_sec = 0.0f;
            h->state = THERMAL_STATE_AUTOTUNE_RUNNING;
            ESP_LOGI(TAG_THERMAL, "[%s] Autotune started at %u C", h->name, (unsigned)cmd->temp_target);
            break;
        }

        default:
            ESP_LOGW(TAG_THERMAL, "Unknown thermal command type: %lu", (unsigned long)cmd->cmd_num);
            break;
    }
}

void thermal_process_command(heater_channel_t heaters[]) {
    thermal_cmd_t cmd;

    /* Drain every pending command this cycle so none pile up behind the 100 ms loop */
    while (xQueueReceive(thermal_cmds_queue, &cmd, 0) == pdTRUE) {
        apply_one_command(heaters, &cmd);

        // Clear "at target" before announcing the command as applied, so the
        // planner can never pair the new seq with a stale "reached" flag.
        thermal_publish_status(heaters);
        s_last_applied_seq = cmd.seq;
    }
}
