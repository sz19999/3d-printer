#include "thermal_task.h"
#include "heaters_fans.h"
#include "adc_filter.h"
#include "thermistor.h"
#include "adc.h"
#include "pid_controller.h"
#include "pid_autotune.h"
#include "motion_planner.h"
#include "esp_log.h"
#include "nvs.h"
#include <math.h>

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
static volatile bool     s_autotune[2]  = {false, false};

/* Conservative starting gains for a 10-bit output (0..1023) at a 100 ms loop.
   Not tuned for this hardware; they favour a slow, low-overshoot approach. */
static const pid_gains_t  HOTEND_DEFAULT_GAINS  = { .kp = 60.0f, .ki = 2.0f, .kd = 250.0f };
static const pid_limits_t HOTEND_DEFAULT_LIMITS = { .min_output = 0.0f, .max_output = 1023.0f, .windup_guard = 15.0f };
static const pid_gains_t  BED_DEFAULT_GAINS     = { .kp = 50.0f, .ki = 1.0f, .kd = 0.0f };
static const pid_limits_t BED_DEFAULT_LIMITS    = { .min_output = 0.0f, .max_output = 1023.0f, .windup_guard = 10.0f };

/* ---- Autotuned gains in NVS flash (survive reboot and reflash) ---- */
#define PID_NVS_NAMESPACE "thermal"
static const char *const PID_NVS_KEYS[2] = { [HEATER_HOTEND] = "pid_hotend", [HEATER_BED] = "pid_bed" };

static bool gains_are_valid(const pid_gains_t *g) {
    return isfinite(g->kp) && isfinite(g->ki) && isfinite(g->kd) &&
           g->kp > 0.0f && g->ki >= 0.0f && g->kd >= 0.0f;
}

/* Returns false (and leaves *out untouched) when nothing valid is stored. */
static bool load_saved_gains(heater_source_t heater, pid_gains_t *out) {
    nvs_handle_t nvs;
    // The namespace does not exist until the first autotune saves into it.
    if (nvs_open(PID_NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        return false;
    }

    pid_gains_t gains;
    size_t len = sizeof(gains);
    esp_err_t err = nvs_get_blob(nvs, PID_NVS_KEYS[heater], &gains, &len);
    nvs_close(nvs);

    if (err != ESP_OK || len != sizeof(gains) || !gains_are_valid(&gains)) {
        return false;
    }
    *out = gains;
    return true;
}

bool thermal_save_gains(heater_source_t heater, const pid_gains_t *gains) {
    if (!gains_are_valid(gains)) {
        ESP_LOGE(TAG_THERMAL, "Refusing to save invalid PID gains to flash");
        return false;
    }

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(PID_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        err = nvs_set_blob(nvs, PID_NVS_KEYS[heater], gains, sizeof(*gains));
        if (err == ESP_OK) {
            err = nvs_commit(nvs);
        }
        nvs_close(nvs);
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG_THERMAL, "Saving PID gains to flash failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

static void init_pid_gains(heater_channel_t *h, heater_source_t heater,
                           const pid_gains_t *defaults, const pid_limits_t *limits) {
    pid_gains_t gains = *defaults;
    bool from_flash = load_saved_gains(heater, &gains);
    pid_init(&h->pid, &gains, limits);
    ESP_LOGI(TAG_THERMAL, "[%s] PID from %s: Kp %.2f, Ki %.2f, Kd %.2f",
             h->name, from_flash ? "flash (autotuned)" : "built-in defaults",
             gains.kp, gains.ki, gains.kd);
}

void thermal_heaters_init(heater_channel_t heaters[]) {
    heater_channel_t *hot = &heaters[HEATER_HOTEND];
    init_pid_gains(hot, HEATER_HOTEND, &HOTEND_DEFAULT_GAINS, &HOTEND_DEFAULT_LIMITS);
    hot->max_target    = HOTEND_MAX_TARGET;
    hot->max_temp      = HOTEND_MAXTEMP;
    hot->target_window = TEMP_WINDOW_HOTEND;
    hot->runaway_period_sec   = HOTEND_RUNAWAY_PERIOD_SEC;
    hot->hold_band            = HOTEND_HOLD_BAND_C;
    hot->hold_period_sec      = HOTEND_HOLD_PERIOD_SEC;
    hot->autotune_cycles      = HOTEND_AUTOTUNE_CYCLES;
    hot->autotune_timeout_sec = HOTEND_AUTOTUNE_TIMEOUT_SEC;

    heater_channel_t *bed = &heaters[HEATER_BED];
    init_pid_gains(bed, HEATER_BED, &BED_DEFAULT_GAINS, &BED_DEFAULT_LIMITS);
    bed->max_target    = BED_MAX_TARGET;
    bed->max_temp      = BED_MAXTEMP;
    bed->target_window = TEMP_WINDOW_BED;
    bed->runaway_period_sec   = BED_RUNAWAY_PERIOD_SEC;
    bed->hold_band            = BED_HOLD_BAND_C;
    bed->hold_period_sec      = BED_HOLD_PERIOD_SEC;
    bed->autotune_cycles      = BED_AUTOTUNE_CYCLES;
    bed->autotune_timeout_sec = BED_AUTOTUNE_TIMEOUT_SEC;

    for (int i = 0; i < 2; i++) {
        heaters[i].state = THERMAL_STATE_OFF;
        heaters[i].runaway_timer_sec = 0.0f;
        heaters[i].residency_sec = 0.0f;
        heaters[i].reached_target = false;
        heaters[i].below_band_sec = 0.0f;
    }
}

void thermal_set_fault(void) {
    // Shut down (and log) once. The thermal loop keeps writing duty 0 on every cycle
    // while the fault is latched, so the outputs stay forced off without log spam.
    if (!s_fault) {
        ESP_LOGE(TAG_THERMAL, "!!! THERMAL FAULT LATCHED - all heaters OFF until reboot !!!");
        s_fault = true;
        mosfet_emergency_shutdown();
    }
}

float thermal_active_target(const heater_channel_t *h) {
    switch (h->state) {
        case THERMAL_STATE_PID_RUNNING:      return h->pid.setpoint;
        case THERMAL_STATE_AUTOTUNE_RUNNING: return h->autotune.config.target_temp;
        default:                             return 0.0f;
    }
}

bool thermal_fault_active(void) { return s_fault; }
uint32_t thermal_last_applied_seq(void) { return s_last_applied_seq; }
bool thermal_at_target(heater_source_t heater) { return s_at_target[heater]; }
bool thermal_autotune_active(heater_source_t heater) { return s_autotune[heater]; }
float thermal_get_temp(heater_source_t heater) { return s_temp[heater]; }
float thermal_get_target(heater_source_t heater) { return s_target[heater]; }

void thermal_publish_status(const heater_channel_t heaters[]) {
    for (int i = 0; i < 2; i++) {
        s_temp[i] = heaters[i].current_temp;
        s_target[i] = thermal_active_target(&heaters[i]);
        s_at_target[i] = (heaters[i].state == THERMAL_STATE_PID_RUNNING) &&
                         (heaters[i].residency_sec >= TEMP_RESIDENCY_SEC);
        s_autotune[i] = (heaters[i].state == THERMAL_STATE_AUTOTUNE_RUNNING);
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
    } else if (cmd->cmd_num == 303 && cmd->autotune_bed) {
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
            h->reached_target = false;     // new target: back to the heating-up runaway rule
            h->below_band_sec = 0.0f;

            if (target > 0.0f) {
                h->state = THERMAL_STATE_PID_RUNNING;
                ESP_LOGI(TAG_THERMAL, "[%s] Target temp updated to %.1f C", h->name, target);
            } else {
                h->state = THERMAL_STATE_OFF;
                ESP_LOGI(TAG_THERMAL, "[%s] Powered down (target 0 C)", h->name);
            }
            break;
        }

        case 303: { /* PID Autotune: M303 E0 S<temp> hotend, M303 E-1 S<temp> bed */
            float target = (float)cmd->temp_target;

            // Same PLA limit as M104/M140: the relay overshoots above target, so never tune higher.
            if (target > h->max_target) {
                ESP_LOGW(TAG_THERMAL, "[%s] Autotune target %.0f C above limit, clamped to %.0f C",
                         h->name, target, h->max_target);
                target = h->max_target;
            }

            autotune_config_t config = {
                .target_temp = target,
                .output_power = 1023.0f,
                .hysteresis = 0.5f,
                .requested_cycles = h->autotune_cycles,
                .timeout_seconds = h->autotune_timeout_sec,
                .method = (idx == HEATER_BED) ? TUNING_METHOD_TYREUS_LUYBEN : TUNING_METHOD_ZIEGLER_NICHOLS
            };

            h->residency_sec = 0.0f;
            adc_filter_reset(&h->filter);
            autotune_init(&h->autotune, &config);
            h->runaway_timer_sec = 0.0f;
            h->reached_target = false;     // new target: back to the heating-up runaway rule
            h->below_band_sec = 0.0f;
            h->state = THERMAL_STATE_AUTOTUNE_RUNNING;
            ESP_LOGI(TAG_THERMAL, "[%s] Autotune started at %.0f C (%d cycles, timeout %.0f s)",
                     h->name, target, config.requested_cycles, config.timeout_seconds);
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
