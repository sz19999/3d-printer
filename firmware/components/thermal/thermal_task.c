#include "thermal_task.h"
#include "heaters_fans.h"
#include "adc_filter.h"
#include "thermistor.h"
#include "adc.h"
#include "pid_controller.h"
#include "pid_autotune.h"
#include "esp_log.h"

static const char *TAG = "THERMAL_TASK";

QueueHandle_t xThermalCommandQueue = NULL;

typedef enum {
    THERMAL_STATE_OFF,
    THERMAL_STATE_PID_RUNNING,
    THERMAL_STATE_AUTOTUNE_RUNNING,
    THERMAL_STATE_FAULT
} thermal_state_t;

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
    const char       *name;
} heater_channel_t;

/* Instantiate per-channel contexts */
static heater_channel_t heaters[2] = {
    [0] = { .name = "HOTEND", .pwm_chan = PWM_CHANNEL_HOTEND, .adc_chan = HOTEND_ADC_CHANNEL },
    [1] = { .name = "BED",    .pwm_chan = PWM_CHANNEL_HEATBED,    .adc_chan = BED_ADC_CHANNEL }
};

#define HEATER_HOTEND 0
#define HEATER_BED    1

#define RUNAWAY_TEMP_THRESHOLD  2.0f    /* Temp must rise 2°C... */
#define RUNAWAY_TIME_LIMIT_SEC  20.0f   /* ...within 20s at high power (>800 raw) */

static void process_incoming_commands(void) {
    thermal_cmd_t cmd;

    while (xQueueReceive(xThermalCommandQueue, &cmd, 0) == pdTRUE) {
        /* Determine targeted channel index (0 = Hotend, 1 = Bed) */
        int channel;

        switch (cmd.cmd_num) {
            case 104:
            case 109:
                channel = 1;
                break;
            case 140:
            case 190:
                channel = 0;
                break;
        }

        uint8_t idx = (channel == HEATER_BED) ? HEATER_BED : HEATER_HOTEND;
        heater_channel_t *h = &heaters[idx];

        if (h->state == THERMAL_STATE_FAULT && cmd.type != THERMAL_CMD_HEATER_OFF) {
            ESP_LOGW(TAG, "[%s] Rejected command: Channel in FAULT!", h->name);
            continue;
        }

        switch (cmd.type) {
            case THERMAL_CMD_SET_TARGET_TEMP:
                h->pid.setpoint = cmd.payload.target_temp;
                adc_filter_reset(&h->filter);
                h->runaway_timer_sec = 0.0f;

                if (cmd.payload.target_temp > 0.0f) {
                    h->state = THERMAL_STATE_PID_RUNNING;
                    ESP_LOGI(TAG, "[%s] Target temp updated to %.1f C", h->name, cmd.payload.target_temp);
                } else {
                    h->state = THERMAL_STATE_OFF;
                    ESP_LOGI(TAG, "[%s] Powered down (target 0 C)", h->name);
                }
                break;

            case THERMAL_CMD_START_AUTOTUNE: {
                autotune_config_t config = {
                    .target_temp = cmd.payload.autotune.target_temp,
                    .output_power = 1023.0f,
                    .hysteresis = 0.5f,
                    .requested_cycles = 5,
                    .timeout_seconds = 300.0f,
                    .method = cmd.payload.autotune.method
                };
                
                adc_filter_reset(&h->filter);
                autotune_init(&h->autotune, &config);
                h->runaway_timer_sec = 0.0f;
                h->state = THERMAL_STATE_AUTOTUNE_RUNNING;
                ESP_LOGI(TAG, "[%s] Autotune started at %.1f C", h->name, cmd.payload.autotune.target_temp);
                break;
            }

            case THERMAL_CMD_SET_GAINS:
                h->pid.kp = cmd.payload.pid_gains.kp;
                h->pid.ki = cmd.payload.pid_gains.ki;
                h->pid.kd = cmd.payload.pid_gains.kd;
                ESP_LOGI(TAG, "[%s] PID Gains updated: Kp=%.2f, Ki=%.2f, Kd=%.2f",
                         h->name, h->pid.kp, h->pid.ki, h->pid.kd);
                break;

            case THERMAL_CMD_HEATER_OFF:
                h->state = THERMAL_STATE_OFF;
                mosfet_set_duty_raw(0.0f, h->pwm_chan);
                ESP_LOGI(TAG, "[%s] Turned OFF", h->name);
                break;

            default:
                ESP_LOGW(TAG, "Unknown thermal command type: %d", cmd.type);
                break;
        }
    }
}

void vThermalTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(100);
    const float dt_seconds = 0.100f;

    xThermalCommandQueue = xQueueCreate(10, sizeof(thermal_cmd_t));
    if (xThermalCommandQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create thermal command queue!");
        vTaskDelete(NULL);
        return;
    }

    dual_adc_t sensors_adc;
    adc_filter_init(&heaters[HEATER_HOTEND].filter, 0.15f);
    adc_filter_init(&heaters[HEATER_BED].filter, 0.1f);  /* Semicolon fixed */
    dual_adc_init(&sensors_adc);
    
    extern thermistor_config_t th_config;
    ESP_LOGI(TAG, "Thermal Control Task online (Core %d)", xPortGetCoreID());

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        /* Step 1: Command Processing */
        process_incoming_commands();

        /* Process control and safety loop for each channel independently */
        for (int i = 0; i < 2; i++) {
            heater_channel_t *h = &heaters[i];

            /* Step 2: Read Sensor Signal */
            uint32_t raw_mv = 0;
            esp_err_t err = dual_adc_read_channel_mv(&sensors_adc, h->adc_chan, &raw_mv);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "[%s] ADC Read Failure! Triggering FAULT.", h->name);
                h->state = THERMAL_STATE_FAULT;
            }

            float smooth_mv = adc_filter_update(&h->filter, (float)raw_mv);
            h->current_temp = thermistor_mv_to_celsius(smooth_mv, &th_config);

            float pwm_output = 0.0f;

            /* Step 3: Execute Control Logic */
            switch (h->state) {
                case THERMAL_STATE_PID_RUNNING:
                    pwm_output = pid_compute(&h->pid, h->current_temp, dt_seconds);
                    
                    if (h->current_temp > (h->pid.setpoint + 20.0f)) {
                        ESP_LOGE(TAG, "[%s] FAULT: Temperature overshot target by >20C!", h->name);
                        h->state = THERMAL_STATE_FAULT;
                    }
                    break;

                case THERMAL_STATE_AUTOTUNE_RUNNING:
                    pwm_output = autotune_step(&h->autotune, h->current_temp, dt_seconds);

                    if (h->autotune.state == AUTOTUNE_STATE_COMPLETE) {
                        ESP_LOGI(TAG, "[%s] Autotune complete. Kp: %.2f, Ki: %.2f, Kd: %.2f",
                                 h->name,
                                 h->autotune.calculated_gains.kp,
                                 h->autotune.calculated_gains.ki,
                                 h->autotune.calculated_gains.kd);

                        pid_init(&h->pid, &h->autotune.calculated_gains, h->autotune.config.target_temp);
                        h->state = THERMAL_STATE_PID_RUNNING;

                    } else if (h->autotune.state == AUTOTUNE_STATE_FAILED) {
                        ESP_LOGE(TAG, "[%s] Autotune failed or timed out!", h->name);
                        h->state = THERMAL_STATE_FAULT;
                    }
                    break;

                case THERMAL_STATE_OFF:
                case THERMAL_STATE_FAULT:
                default:
                    pwm_output = 0.0f;
                    break;
            }

            /* Step 4: Runaway Protection Logic */
            if (pwm_output > 818.0f && h->state != THERMAL_STATE_FAULT) {
                if (h->runaway_timer_sec == 0.0f) {
                    h->runaway_start_temp = h->current_temp;
                }
                
                h->runaway_timer_sec += dt_seconds;
                
                if (h->runaway_timer_sec >= RUNAWAY_TIME_LIMIT_SEC) {
                    if ((h->current_temp - h->runaway_start_temp) < RUNAWAY_TEMP_THRESHOLD) {
                        ESP_LOGE(TAG, "[%s] THERMAL RUNAWAY: Power applied but temp not rising!", h->name);
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

            /* Step 5: Drive Hardware PWM Output */
            if (h->state == THERMAL_STATE_FAULT) {
                mosfet_set_duty_raw(0.0f, h->pwm_chan);
            } else {
                mosfet_set_duty_raw(pwm_output, h->pwm_chan);
            }
        }
    }
}