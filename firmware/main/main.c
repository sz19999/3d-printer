#include <stdio.h>
#include <dirent.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"

#include "gcode_parser.h"
#include "motion_planner.h"
#include "spi_sd.h"
#include "step_generator.h"
#include "sys_state_machine.h"
#include "thermal_task.h"
#include "user_interface.h"
#include "i2c_oled.h"
#include "tmc2209.h"


#define GCODE_LINE_MAX_LEN 128
#define SYS_RUNNING_BIT (1 << 0)

const char *TAG_PARSER  = "PARSER_TASK";
const char *TAG_PLANNER = "PLANNER_TASK";
const char *TAG_MAIN    = "MAIN_APP";
const char *TAG_MOTION  = "MOTION_BLOCK_TASK";
const char *TAG_SD      = "SD_STREAMER_TASK";
const char *TAG_RMT     = "STEP_GENERATOR_TASK";
const char *TAG_SYS     = "SYS_STATE_TASK";
const char *TAG_THERMAL = "THERMAL_TASK";
const char* TAG_UI   = "UI_TASK"; 

// Global handles for queues
QueueHandle_t gcode_line_queue = NULL;
QueueHandle_t gcode_cmds_queue = NULL;
QueueHandle_t motion_queue = NULL;
QueueHandle_t gpio_evt_queue = NULL;
QueueHandle_t thermal_cmds_queue = NULL;
QueueHandle_t ui_queue       = NULL;

TaskHandle_t xParserTaskHandle  = NULL;
TaskHandle_t xSDTaskHandle      = NULL;
TaskHandle_t xPlannerTaskHandle = NULL;
TaskHandle_t xStepGenTaskHandle = NULL;
TaskHandle_t xSysStateTaskHandle = NULL;
TaskHandle_t xThermalTaskHandle = NULL;
TaskHandle_t xUITaskHandle       = NULL;

EventGroupHandle_t sys_event_group;

// Live status for the OLED status screen. Each variable has exactly one writer
// task; the UI task only reads them. 32-bit loads/stores are atomic on the S3.
static volatile long  g_file_size        = 0;      // SD streamer
static volatile long  g_file_bytes_read  = 0;      // SD streamer
static volatile bool  g_file_done        = false;  // SD streamer
static volatile bool  g_waiting_for_heat = false;  // planner (inside M109/M190)
static volatile bool  g_block_executing  = false;  // step generator
static volatile bool  g_homing_block     = false;  // step generator
static volatile float g_pos_mm[3]        = {0};    // step generator: X/Y/Z after last finished block

void print_motion_block(const PlannedMotion* block);

void sd_streamer_task(void *pvParameters) {
    ESP_LOGI(TAG_SD, "Task started successfully on core %d", xPortGetCoreID());

    // block here until the bit is set.
    xEventGroupWaitBits(
        sys_event_group,
        SYS_RUNNING_BIT,
        pdFALSE,        // Don't clear bit on exit (so other tasks stay unblocked)
        pdTRUE,         // Wait for all bits
        portMAX_DELAY   // Wait indefinitely
    );

    // init & mount sd card
    sdmmc_card_t* card = NULL;
    sdmmc_host_t host  = SDSPI_HOST_DEFAULT();
    sd_init(&card, &host);

    // find the first gcode file in the sd card
    FILE* f = open_first_by_extension(MOUNT_POINT, ".gcode", "rb");
    if (f == NULL) {
        ESP_LOGE(TAG_SD, "Couldn't open gcode file!");
    }
    else {
        ESP_LOGI(TAG_SD, "Opened gcode file scuccessfuly!");

        // file size for the progress display
        if (fseek(f, 0, SEEK_END) == 0) {
            g_file_size = ftell(f);
            fseek(f, 0, SEEK_SET);
        }
        g_file_bytes_read = 0;
        g_file_done = false;

        while(1) {
            xEventGroupWaitBits(
                sys_event_group,
                SYS_RUNNING_BIT,
                pdFALSE,        // Don't clear bit on exit (so other tasks stay unblocked)
                pdTRUE,         // Wait for all bits
                portMAX_DELAY   // Wait indefinitely
            );

            char gcode_line[GCODE_LINE_MAX_LEN];

            // read gcode line
            if (fgets(gcode_line, sizeof(gcode_line), f) == NULL) {
                ESP_LOGI(TAG_SD, "Reached to the end of the gcode file!");
                g_file_done = true;
                fclose(f);
                vTaskDelay(pdMS_TO_TICKS(1000000)); 
                continue;
            }

            g_file_bytes_read += strlen(gcode_line);

            // dispatch gcode line
            if (xQueueSend(gcode_line_queue, gcode_line, portMAX_DELAY) == pdPASS) { // Wait indefinitely if queue is full
                ESP_LOGI(TAG_SD, "Dispatched gcode line to gcode_line_queue.");
            }
            vTaskDelay(1); 
           // vTaskDelay(pdMS_TO_TICKS(1000)); // Brief delay for smooth serial observation
        }

        // teardown & clean unmount
        esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
    }
}

void parser_task(void *pvParameters) {
    char gcode_line[GCODE_LINE_MAX_LEN];
    GCodeCommand gcode_cmd;

    ESP_LOGI(TAG_PARSER, "Task started successfully on core %d", xPortGetCoreID());

    while (1) {
        xEventGroupWaitBits(
                sys_event_group,
                SYS_RUNNING_BIT,
                pdFALSE,        // Don't clear bit on exit (so other tasks stay unblocked)
                pdTRUE,         // Wait for all bits
                portMAX_DELAY   // Wait indefinitely
            );

        // Wait indefinitely for raw gcode string lines
        if (xQueueReceive(gcode_line_queue, gcode_line, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG_PARSER, "Received raw line: \"%s\"", gcode_line);

            // Parse raw gcode line text buffer
            if (!parse_command(gcode_line, &gcode_cmd)) {
                ESP_LOGE(TAG_PARSER, "Failed to parse command: \"%s\"", gcode_line);
                continue;   // dont dispatch invalid commands
            }

            // dont dispatch pure comments
            if (gcode_line[0] == ';') continue;

            ESP_LOGI(TAG_PARSER, "Successfully parsed G-Code command: \"%s\".", gcode_line);

            // Dispatch into the parsed gcode queue
            if (xQueueSend(gcode_cmds_queue, &gcode_cmd, portMAX_DELAY) == pdPASS) {
                ESP_LOGI(TAG_PARSER, "Dispatched GCodeCommand to gcode_cmds_queue.");
            } else {
                ESP_LOGE(TAG_PARSER, "gcode_cmds_queue full! Command dropped.");
            }
        }
        vTaskDelay(1);
       // vTaskDelay(pdMS_TO_TICKS(1000)); // Brief delay for smooth serial observation
    }
}

// Send every block still held in the lookahead buffer to the step generator.
static void flush_planner_buffer(RingBuffer *buffer) {
    PlannedMotion *m;
    while ((m = front(buffer)) != NULL) {
        if (xQueueSend(motion_queue, m, portMAX_DELAY) == pdPASS) {
            print_motion_block(m);
            pop(buffer);
        }
    }
}

// Block until the heater reports "at target" for the command stamped `seq`.
// Returns early on a thermal fault or when the print is aborted.
static void planner_wait_for_temperature(heater_source_t heater, uint32_t seq) {
    const char *name = (heater == HEATER_BED) ? "BED" : "HOTEND";
    ESP_LOGI(TAG_PLANNER, "Waiting for %s to reach target...", name);

    g_waiting_for_heat = true;   // shown as HEATING on the display

    TickType_t last_log = xTaskGetTickCount();
    while (1) {
        if (thermal_fault_active()) {
            ESP_LOGE(TAG_PLANNER, "Stopped waiting for %s: thermal fault.", name);
            break;
        }
        if ((xEventGroupGetBits(sys_event_group) & SYS_RUNNING_BIT) == 0) {
            ESP_LOGW(TAG_PLANNER, "Stopped waiting for %s: print aborted.", name);
            break;
        }
        // Only trust "at target" once the thermal task has applied THIS command.
        if ((int32_t)(thermal_last_applied_seq() - seq) >= 0 && thermal_at_target(heater)) {
            ESP_LOGI(TAG_PLANNER, "%s at target (%.1f C). Resuming.", name, thermal_get_temp(heater));
            break;
        }
        if (xTaskGetTickCount() - last_log >= pdMS_TO_TICKS(5000)) {
            last_log = xTaskGetTickCount();
            ESP_LOGI(TAG_PLANNER, "...%s at %.1f C, still heating", name, thermal_get_temp(heater));
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    g_waiting_for_heat = false;
}

// Same effect as the UI's Abort, triggered by a latched thermal fault.
static void abort_print_on_thermal_fault(RingBuffer *buffer) {
    ESP_LOGE(TAG_PLANNER, "THERMAL FAULT - aborting print.");
    xEventGroupClearBits(sys_event_group, SYS_RUNNING_BIT);
    vTaskSuspend(xStepGenTaskHandle);
    xQueueReset(gcode_line_queue);
    xQueueReset(gcode_cmds_queue);
    xQueueReset(motion_queue);
    xQueueReset(thermal_cmds_queue);
    init_buffer(buffer);
}

void motion_planner_task(void *pvParameters) {
    uint32_t thermal_seq = 0;
    GCodeCommand gcode_cmd;
    PlannedMotion* motion = NULL;
    RingBuffer buffer;

    PointMM current_mm = {0.0f, 0.0f, 0.0f, 0.0f};
    PointSteps current_steps = {0, 0, 0, 0};
    bool absolute_mode = true;
    PlannerState planner_state = PLANNER_STATE_BUFFERING;

    init_buffer(&buffer);
    ESP_LOGI(TAG_PLANNER, "Task started successfully on core %d", xPortGetCoreID());
    
    while (1) {
        xEventGroupWaitBits(
                sys_event_group,
                SYS_RUNNING_BIT,
                pdFALSE,        // Don't clear bit on exit (so other tasks stay unblocked)
                pdTRUE,         // Wait for all bits
                portMAX_DELAY   // Wait indefinitely
            );

        // A latched thermal fault stops the print; never keep moving with dead heaters.
        if (thermal_fault_active()) {
            abort_print_on_thermal_fault(&buffer);
            planner_state = PLANNER_STATE_BUFFERING;
            continue;   // the cleared running bit parks this task at the wait above
        }

        memset(&gcode_cmd, 0, sizeof(GCodeCommand)); // reset gcode cmd holder

        if (xQueueReceive(gcode_cmds_queue, &gcode_cmd, pdMS_TO_TICKS(10)) == pdTRUE) {
            ESP_LOGI(TAG_PLANNER, "Received parsed command from queue.");

            if (is_motion_command(&gcode_cmd)) {
                ESP_LOGI(TAG_PLANNER, "Command identified as MOTION. Planning velocity profile...");

                handle_motion_command(&gcode_cmd, &buffer, &current_mm, &current_steps, &absolute_mode);

                if (buffer.count >= MIN_PLANNER_BLOCKS) {
                    planner_state = PLANNER_STATE_RUNNING;
                } else {
                    ESP_LOGW(TAG_PLANNER, "Buffer still buffering. Elements count: %d", buffer.count);
                }
            } else {
                ESP_LOGI(TAG_PLANNER, "Command identified as NON-MOTION (Heater/Fan/State). Processing metadata...");
                thermal_cmd_t metadata;
                memset(&metadata, 0, sizeof(thermal_cmd_t));
                handle_metadata_command(&gcode_cmd, &metadata);

                if (metadata.cmd_num == 0) {
                    // Not a heater/fan command we handle (e.g. M82, M84): nothing to send.
                } else {
                    metadata.seq = ++thermal_seq;

                    // The thermal task drains the whole queue every 100 ms, so a
                    // bounded wait here never stalls the pipeline for long.
                    if (xQueueSend(thermal_cmds_queue, &metadata, pdMS_TO_TICKS(1000)) == pdPASS) {
                        ESP_LOGI(TAG_PLANNER, "Dispatched M%lu to thermal_cmds_queue.",
                                 (unsigned long)metadata.cmd_num);
                    } else {
                        ESP_LOGE(TAG_PLANNER, "thermal_cmds_queue full for 1 s, dropped M%lu!",
                                 (unsigned long)metadata.cmd_num);
                    }

                    // M109 / M190: block the G-code stream until the heater is at target.
                    if ((metadata.cmd_num == 109 || metadata.cmd_num == 190) && metadata.temp_target > 0) {
                        // Moves planned before the wait must run now, not after heating.
                        flush_planner_buffer(&buffer);
                        planner_state = PLANNER_STATE_BUFFERING;

                        heater_source_t heater = (metadata.cmd_num == 190) ? HEATER_BED : HEATER_HOTEND;
                        planner_wait_for_temperature(heater, metadata.seq);
                    }
                }
            }
        }
        else {
            // flush remaining commands
            if (!is_empty(&buffer)) {
                planner_state = PLANNER_STATE_RUNNING;
            } 
        }

        if (planner_state == PLANNER_STATE_RUNNING) {
            // dispatch oldest motion to the step generator
            //ESP_LOGI(TAG_PLANNER, "current position in mm: (%f, %f, %f, %f)", current_mm.x, current_mm.y, current_mm.z, current_mm.e);
            
            motion = front(&buffer);
            if (motion != NULL) {
                
                if (xQueueSend(motion_queue, motion, portMAX_DELAY) == pdPASS) {
                    ESP_LOGI(TAG_PLANNER, "Buffer front returned a motion block.");
                    print_motion_block(motion);

                    ESP_LOGI(TAG_PLANNER, "Dispatched PlannedMotion block to motion_queue.");
                    pop(&buffer);
                } else {
                    ESP_LOGE(TAG_PLANNER, "motion_queue full! Could not send motion block.");
                }

            }
        }

        if (is_empty(&buffer)) {
            planner_state = PLANNER_STATE_BUFFERING;
        } 
        vTaskDelay(1);
       // vTaskDelay(pdMS_TO_TICKS(1000));
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
    uint32_t log_ticks = 0;

    thermal_heaters_init(heaters);
    adc_filter_init(&heaters[HEATER_HOTEND].filter, 0.15f);
    adc_filter_init(&heaters[HEATER_BED].filter, 0.1f);
    if (dual_adc_init(&sensors_adc) != ESP_OK) {
        ESP_LOGE(TAG_THERMAL, "ADC init failed - heaters disabled.");
        thermal_set_fault();
    }

    ESP_LOGI(TAG_THERMAL, "Thermal Control Task online (Core %d)", xPortGetCoreID());

    // This loop runs ALWAYS, independent of SYS_RUNNING_BIT. It used to block on
    // that bit, which on Abort froze the heaters at their last PWM duty with no
    // control or protection. Safety checks must never stop running.
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        /* Step 1: Command Processing (Heater & Fan Queue) */
        thermal_process_command(heaters);

        /* Step 2: Explicit rule - heaters may only run while a print is running.
           Abort clears SYS_RUNNING_BIT; Pause keeps it set so the hotend stays warm. */
        bool print_running = (xEventGroupGetBits(sys_event_group) & SYS_RUNNING_BIT) != 0;
        if (!print_running) {
            for (int i = 0; i < 2; i++) {
                if (heaters[i].state == THERMAL_STATE_PID_RUNNING ||
                    heaters[i].state == THERMAL_STATE_AUTOTUNE_RUNNING) {
                    ESP_LOGW(TAG_THERMAL, "[%s] Print not running - heater OFF", heaters[i].name);
                    heaters[i].state = THERMAL_STATE_OFF;
                    heaters[i].pid.setpoint = 0.0f;
                    heaters[i].residency_sec = 0.0f;
                }
            }
        }

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

            /* Absolute limits, checked in EVERY state (a failed-on MOSFET heats
               even while the heater is OFF). Any violation latches a global fault. */
            if (isnan(h->current_temp) || h->current_temp < THERMAL_MINTEMP) {
                if (h->state != THERMAL_STATE_FAULT) {
                    ESP_LOGE(TAG_THERMAL, "[%s] MINTEMP: reading %.1f C (sensor shorted/broken?)",
                             h->name, h->current_temp);
                }
                h->state = THERMAL_STATE_FAULT;
                thermal_set_fault();
            } else if (h->current_temp > h->max_temp) {
                if (h->state != THERMAL_STATE_FAULT) {
                    ESP_LOGE(TAG_THERMAL, "[%s] MAXTEMP: %.1f C exceeds %.0f C limit!",
                             h->name, h->current_temp, h->max_temp);
                }
                h->state = THERMAL_STATE_FAULT;
                thermal_set_fault();
            }

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

            /* "Target reached" timer for M109 / M190 */
            if (h->state == THERMAL_STATE_PID_RUNNING &&
                fabsf(h->current_temp - h->pid.setpoint) <= h->target_window) {
                h->residency_sec += dt_seconds;
            } else {
                h->residency_sec = 0.0f;
            }

            /* A fault on either heater stops both (latched until reboot). */
            if (h->state == THERMAL_STATE_FAULT) {
                thermal_set_fault();
            }

            /* Drive Hardware PWM Output (signature is: channel, duty) */
            if (thermal_fault_active() || h->state == THERMAL_STATE_FAULT) {
                mosfet_set_duty_raw(h->pwm_chan, 0);
            } else {
                mosfet_set_duty_raw(h->pwm_chan, (uint32_t)pwm_output);
            }
        }

        thermal_publish_status(heaters);

        /* Temperature report once per second */
        if (++log_ticks >= 10) {
            log_ticks = 0;
            ESP_LOGI(TAG_THERMAL, "HOTEND %.1f/%.0f C | BED %.1f/%.0f C%s",
                     heaters[HEATER_HOTEND].current_temp, heaters[HEATER_HOTEND].pid.setpoint,
                     heaters[HEATER_BED].current_temp, heaters[HEATER_BED].pid.setpoint,
                     thermal_fault_active() ? " | FAULT" : "");
        }
    }
}

// Endstop input pin per axis id (X, Y, Z), for level checks from the step task.
static const gpio_num_t k_endstop_pins[3] = { ENDSTOP_X_GPIO, ENDSTOP_Y_GPIO, ENDSTOP_Z_GPIO };

// True only if the (active-LOW) endstop reads pressed on every one of 3 samples
// taken 20 us apart, so a single coupled-noise spike can't fake a touch.
// Needed because the endstop interrupt is edge-triggered: if the ISR rejects the
// first edge as bounce, a switch that stays pressed never produces another edge.
static bool endstop_is_pressed(uint8_t axis) {
    for (int i = 0; i < 3; i++) {
        if (gpio_get_level(k_endstop_pins[axis]) != 0) return false;
        esp_rom_delay_us(20);
    }
    return true;
}

void step_generator_task(void *pvParameters) {
    const uint8_t step_pins[NUM_AXES] = {X_STEP_PIN, Y_STEP_PIN, Z_STEP_PIN, E_STEP_PIN};
    const uint8_t dir_pins[NUM_AXES]  = {X_DIR_PIN,  Y_DIR_PIN,  Z_DIR_PIN,  E_DIR_PIN};

    PlannedMotion motion;
    multi_axis_dda_generator_t dda;
    rmt_stepper_system_t sys = {0};

    // Ping-pong DRAM buffers: 2 Banks x 4 Axes x 64 Symbols
    static rmt_symbol_word_t ping_pong_buff[2][NUM_AXES][SYMBOLS_PER_BLOCK];
    
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };


    ESP_LOGI(TAG_RMT, "Step Generator Task started on Core %d", xPortGetCoreID());

    // initialize axes endstop switches
    init_endstops();

    // bring up TMC2209 UART bus and configure each driver (current, microstepping)
    tmc2209_init_uart();
    setup_tmc2209(TMC2209_X_ADDR);
    setup_tmc2209(TMC2209_Y_ADDR);
    setup_tmc2209(TMC2209_Z_ADDR);
    setup_tmc2209(TMC2209_E_ADDR);

    // initialize RMT Hardware Channels
    init_stepper_rmt_channels(&sys, step_pins);

    // register callback only on Master Channel
    register_stepper_callbacks(&sys, xTaskGetCurrentTaskHandle());
    
    // configure direction pins as outputs
    for (int i = 0; i < NUM_AXES; i++) {
        gpio_reset_pin(dir_pins[i]);
        gpio_set_direction(dir_pins[i], GPIO_MODE_OUTPUT);
    }

    vTaskSuspend(xStepGenTaskHandle);

    while (1) {
        // wait for next motion block from motion planner queue
        if (xQueueReceive(motion_queue, &motion, portMAX_DELAY) == pdTRUE) {
            bool abort_triggered = false;
            uint32_t notify_value = 0;

            ESP_LOGI(TAG_RMT, "Received motion block -> master_steps: %lu, dir_bits: 0x%02X", 
                     (unsigned long)motion.master_steps, motion.dir_bits);

            g_homing_block = (motion.motion_mode == MOTION_MODE_HOMING);
            g_block_executing = true;

            // set direction pins
            for (int i = 0; i < NUM_AXES; i++) {
                gpio_set_level(dir_pins[i], (motion.dir_bits >> i) & 0x01);
            }

            // initialize DDA state machine for new move block
            dda.block = motion;
            dda.master_step_count = 0;
            memset(dda.accumulators, 0, sizeof(dda.accumulators));

            // Arm endstops only for a homing move whose master axis is driving
            // TOWARD its switch (negative dir bit). Back-off moves, non-master
            // axes, and print moves get their endstop interrupt disabled so
            // coupled motor noise can't storm the CPU.
            bool homing_approach = false;
            uint8_t approach_axis = motion.master_axis;
            if (motion.motion_mode == MOTION_MODE_HOMING && approach_axis < 3 &&
                ((motion.dir_bits >> approach_axis) & 0x01)) {
                homing_approach = true;
            }
            for (uint8_t a = 0; a < 3; a++) {
                endstop_set_armed(a, homing_approach && a == approach_axis);
            }

            // Explicit case: the carriage is already sitting on its switch when the
            // approach starts. Skip the approach; the next queued block is the back-off.
            if (homing_approach && endstop_is_pressed(approach_axis)) {
                ESP_LOGW(TAG_RMT, "Axis %d already on its endstop, skipping approach", approach_axis);
                endstop_set_armed(approach_axis, false);
                g_pos_mm[approach_axis] = 0.0f;
                g_block_executing = false;
                continue;
            }

            // Flush notification bits latched since the previous block (boot-time
            // pin transients, between-block endstop glitches, leftover RMT_TX_DONE)
            // so a stale bit isn't misread as an abort on this block's first wait.
            xTaskNotifyStateClear(NULL);
            ulTaskNotifyValueClear(NULL, ULONG_MAX);

            uint32_t active_bank = 0;
            uint32_t active_transports = 0;

            // prime bank 0 (bank A)
            size_t symbols_written_a = 0;
            generate_dda_rmt_buffers(&dda, ping_pong_buff[0], &symbols_written_a);
            ESP_LOGI(TAG_RMT, "Primed Bank 0: %u symbols generated", (unsigned int)symbols_written_a);

            if (symbols_written_a > 0) {
                for (int axis = 0; axis < NUM_AXES; axis++) {
                    ESP_ERROR_CHECK(rmt_transmit(
                        sys.tx_channels[axis], 
                        sys.copy_encoders[axis],
                        ping_pong_buff[0][axis],
                        symbols_written_a * sizeof(rmt_symbol_word_t),
                        &tx_config
                    ));
                }
                active_transports++; // track queued bank
            }

            // prime bank 1 (bank B) if steps remain in current motion block
            size_t symbols_written_b = 0;
            if (dda.master_step_count < dda.block.master_steps) {
                generate_dda_rmt_buffers(&dda, ping_pong_buff[1], &symbols_written_b);
                ESP_LOGI(TAG_RMT, "Primed Bank 1: %u symbols generated", (unsigned int)symbols_written_b);

                if (symbols_written_b > 0) {
                    for (int axis = 0; axis < NUM_AXES; axis++) {
                        ESP_ERROR_CHECK(rmt_transmit(
                            sys.tx_channels[axis], 
                            sys.copy_encoders[axis],
                            ping_pong_buff[1][axis],
                            symbols_written_b * sizeof(rmt_symbol_word_t),
                            &tx_config
                        ));
                    }
                    active_transports++; // track queued bank
                }
            }

            // streaming loop with endstop abort protection
            uint32_t iterations = 0;
            while (dda.master_step_count < dda.block.master_steps && !abort_triggered) {
                
                // block until either: RMT TX Done fires OR Endstop ISR sends Abort Notification
                if (xTaskNotifyWait(0, ULONG_MAX, &notify_value, portMAX_DELAY) == pdTRUE) {
                    
                    // check if notification came from Endstop ISR (bits 0x0F reserved for axis aborts)
                    if (notify_value & 0x0F) {
                        ESP_LOGE(TAG_RMT, ">>> ABORT SIGNAL RECEIVED (0x%02X) - CANCELLING BLOCK <<<", (unsigned int)notify_value);
                        ESP_LOGE(TAG_RMT, "    live levels X=%d Y=%d Z=%d | ISR raw/low  X=%lu/%lu  Y=%lu/%lu  Z=%lu/%lu",
                                 gpio_get_level(ENDSTOP_X_GPIO), gpio_get_level(ENDSTOP_Y_GPIO), gpio_get_level(ENDSTOP_Z_GPIO),
                                 (unsigned long)g_endstop_raw_isr_hits[0], (unsigned long)g_endstop_confirmed_low[0],
                                 (unsigned long)g_endstop_raw_isr_hits[1], (unsigned long)g_endstop_confirmed_low[1],
                                 (unsigned long)g_endstop_raw_isr_hits[2], (unsigned long)g_endstop_confirmed_low[2]);
                        abort_triggered = true;
                        break; // exit streaming loop immediately
                    }

                    // otherwise, notification is RMT TX_DONE callback
                    active_transports--;

                    // Re-arm the approaching endstop: the ISR self-masks on its
                    // first (usually noise) edge, so without this a real touch
                    // after that point would be missed. Once per refill is
                    // frequent enough for homing feedrates.
                    if (homing_approach) {
                        // Explicit case: the ISR dropped the touch edge as bounce,
                        // but the switch is now held pressed. Treat it as the touch.
                        if (endstop_is_pressed(approach_axis)) {
                            ESP_LOGW(TAG_RMT, "Axis %d touch caught by level check", approach_axis);
                            notify_value = (1u << approach_axis);
                            abort_triggered = true;
                            break;
                        }
                        endstop_set_armed(approach_axis, true);
                    }

                    // refill released bank
                    size_t symbols_written = 0;
                    generate_dda_rmt_buffers(&dda, ping_pong_buff[active_bank], &symbols_written);

                    if (symbols_written > 0) {
                        for (int axis = 0; axis < NUM_AXES; axis++) {
                            ESP_ERROR_CHECK(rmt_transmit(
                                sys.tx_channels[axis], 
                                sys.copy_encoders[axis],
                                ping_pong_buff[active_bank][axis],
                                symbols_written * sizeof(rmt_symbol_word_t),
                                &tx_config
                            ));
                        }
                        active_transports++;
                    }

                    active_bank ^= 1;
                    iterations++;
                }
            }

            // Drain or Emergency Cleanup
            if (!abort_triggered) {
                // Normal Drain Phase: Wait for remaining queued transactions to finish
                while (active_transports > 0) {
                    if (xTaskNotifyWait(0, ULONG_MAX, &notify_value, portMAX_DELAY) == pdTRUE) {
                        if (notify_value & 0x0F) { // Endstop hit during drain
                            abort_triggered = true;
                            break;
                        }
                        active_transports--;

                        // Same level check as the streaming loop: a touch whose
                        // edge was rejected as bounce must still stop the approach.
                        if (homing_approach) {
                            if (endstop_is_pressed(approach_axis)) {
                                ESP_LOGW(TAG_RMT, "Axis %d touch caught by level check (drain)", approach_axis);
                                notify_value = (1u << approach_axis);
                                abort_triggered = true;
                                break;
                            }
                            endstop_set_armed(approach_axis, true);
                        }
                    }
                }
            }
            

            if (abort_triggered) {
                // Decode which axis tripped (explicit, single-bit).
                uint8_t tripped_axis = 0;
                if      (notify_value & ENDSTOP_X_TRIGGERED) tripped_axis = 0;
                else if (notify_value & ENDSTOP_Y_TRIGGERED) tripped_axis = 1;
                else if (notify_value & ENDSTOP_Z_TRIGGERED) tripped_axis = 2;

                // The endstop ISR stopped only the tripped channel via rmt_ll and
                // left a queued ping-pong bank behind; the other channels still
                // hold this block's transactions. Cycle all of them clean.
                //
                // rmt_enable() restarts any transaction still queued, and the copy
                // encoder reads a queued buffer only when its transaction starts.
                // Overwrite both banks with step-free idle symbols first so the
                // leftover bank can't push the carriage further into the switch.
                // Durations must be nonzero: RMT treats 0 as "hold forever".
                const rmt_symbol_word_t idle = {
                    .duration0 = 1, .level0 = 0, .duration1 = 1, .level1 = 0
                };
                for (int b = 0; b < 2; b++) {
                    for (int a = 0; a < NUM_AXES; a++) {
                        for (int i = 0; i < SYMBOLS_PER_BLOCK; i++) {
                            ping_pong_buff[b][a][i] = idle;
                        }
                    }
                }

                for (int axis = 0; axis < NUM_AXES; axis++) {
                    rmt_disable(sys.tx_channels[axis]);
                    rmt_enable(sys.tx_channels[axis]);
                }

                if (motion.motion_mode == MOTION_MODE_HOMING) {
                    ESP_LOGI(TAG_RMT, "Homing touch detected on Axis %d", tripped_axis);
                    // Ignore this switch from now on: the carriage is pressing it,
                    // and the next planned move drives away from it. It re-arms
                    // only when a future homing block approaches it again.
                    endstop_set_armed(tripped_axis, false);
                    // motion_queue NOT flushed -> the planned back-off move runs next
                } else {
                    // PRINTING MODE: Unexpected Crash / Hard Limit
                    ESP_LOGE(TAG_RMT, "CRITICAL: Unexpected endstop trip during print execution!");
                    xQueueReset(motion_queue); // Flush remaining G-code execution stream
                }
            }

            // Publish position for the display. Homing blocks carry pre-home
            // coordinates, and the planner calls the homed spot 0, so explicitly
            // report the homed axis as 0 instead of the block's end_mm.
            if (motion.motion_mode == MOTION_MODE_HOMING) {
                if (motion.master_axis < 3) g_pos_mm[motion.master_axis] = 0.0f;
            } else if (!abort_triggered) {
                g_pos_mm[0] = motion.end_mm[0];
                g_pos_mm[1] = motion.end_mm[1];
                g_pos_mm[2] = motion.end_mm[2];
            }
            g_block_executing = false;

            ESP_LOGI(TAG_RMT, "Motion block execution finished (%u ping-pong refills)", (unsigned int)iterations);
        }
    }
}

void sys_state_machine_task(void *pvParameters) {
    ESP_LOGI(TAG_SYS, "Task started successfully on core %d", xPortGetCoreID());
    
    init_button_interrupt();

    bool first_run = true;
    int selection = 0;
    ui_data_t ui_data_old = {0};
    ui_data_t ui_data     = {0};
    ui_data.screen = MAIN_SCREEN;
    
    while(1) {
        ButtonEvent_t event = process_button_edges();
        
        if (event == EVENT_NONE) {
            if (first_run) {
                if (xQueueSend(ui_queue, &ui_data, pdMS_TO_TICKS(100)) == pdPASS) {
                    ESP_LOGI(TAG_SYS, "Dispatched UI data to turn the display on!");
                }
                first_run = false;
            }
            else {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
        }

        ui_data.button_event = event;

        switch (event) {
            case EVENT_SINGLE_CLICK:
                if (ui_data.screen == MAIN_SCREEN) selection = (selection + 1) % NUM_MAIN_ITEMS;
                else if (ui_data.screen == MENU_SCREEN) selection = (selection + 1) % NUM_MENU_ITEMS;

                ESP_LOGI(TAG_SYS, "Single Click!");
                break;

            case EVENT_DOUBLE_CLICK:
                if (ui_data.screen == MAIN_SCREEN) selection = (NUM_MAIN_ITEMS + selection - 1) % NUM_MAIN_ITEMS;
                else selection = (NUM_MENU_ITEMS + selection - 1) % NUM_MENU_ITEMS;
                ESP_LOGI(TAG_SYS, "Double Click!");
                break;

            case EVENT_LONG_PRESS:
                if (ui_data.screen == MAIN_SCREEN) handle_main_selection(&selection, &ui_data);
                else if (ui_data.screen == MENU_SCREEN) handle_menu_selection(&selection, &ui_data);
                else handle_status_screen_selection(&selection, &ui_data);
                ESP_LOGI(TAG_SYS, "Long Press!");
                break;

            default:
                break;
        }

        ui_data.selection = selection;

        if (!ui_isEqual(&ui_data, &ui_data_old)) {
            if (xQueueSend(ui_queue, &ui_data, pdMS_TO_TICKS(100)) == pdPASS) {
                ESP_LOGI(TAG_SYS, "Dispatched UI Data!");
                ui_data_old = ui_data;
            } else {
                ESP_LOGW(TAG_SYS, "UI Queue Full - Dropped Frame");
            }
        }
    }
}

// Work out what the printer is doing right now, for the status screen.
static ui_print_state_t get_print_state(void) {
    if (thermal_fault_active()) return UI_STATE_FAULT;

    bool running = (xEventGroupGetBits(sys_event_group) & SYS_RUNNING_BIT) != 0;
    if (!running) return UI_STATE_IDLE;

    if (eTaskGetState(xStepGenTaskHandle) == eSuspended) return UI_STATE_PAUSED;
    if (g_waiting_for_heat) return UI_STATE_HEATING;
    if (g_block_executing && g_homing_block) return UI_STATE_HOMING;

    // Done only when the file is fully read AND nothing is left anywhere in the pipeline.
    if (g_file_done && !g_block_executing &&
        uxQueueMessagesWaiting(gcode_line_queue) == 0 &&
        uxQueueMessagesWaiting(gcode_cmds_queue) == 0 &&
        uxQueueMessagesWaiting(motion_queue) == 0) {
        return UI_STATE_DONE;
    }
    return UI_STATE_PRINTING;
}

static void fill_live_status(ui_live_t *live, ui_print_state_t state, uint32_t elapsed_sec) {
    live->state         = state;
    live->nozzle_temp   = thermal_get_temp(HEATER_HOTEND);
    live->nozzle_target = thermal_get_target(HEATER_HOTEND);
    live->bed_temp      = thermal_get_temp(HEATER_BED);
    live->bed_target    = thermal_get_target(HEATER_BED);
    live->x             = g_pos_mm[0];
    live->y             = g_pos_mm[1];
    live->z             = g_pos_mm[2];
    live->elapsed_sec   = elapsed_sec;

    long size = g_file_size;
    if (size > 0) {
        long pct = (long)(((int64_t)g_file_bytes_read * 100) / size);   // 64-bit: no overflow on big files
        if (state == UI_STATE_DONE) pct = 100;
        live->progress = (int)(pct > 100 ? 100 : pct);
    } else {
        live->progress = -1;
    }
}

void ui_task(void *pvParameters) {
    ESP_LOGI(TAG_UI, "Task started successfully on core %d", xPortGetCoreID());

    oled_init();

    const TickType_t refresh_period = pdMS_TO_TICKS(500);   // status screen refresh: 2 Hz

    ui_data_t ui_data = {0};
    ui_data.screen = MAIN_SCREEN;
    bool have_ui_data = false;

    // print timer: starts when a print starts, freezes when it ends
    bool       timer_running = false;
    TickType_t print_start   = 0;
    uint32_t   elapsed_sec   = 0;

    while(1) {
        bool got_event = (xQueueReceive(ui_queue, &ui_data, refresh_period) == pdTRUE);
        if (got_event) {
            ESP_LOGI(TAG_UI, "Received UI data!");
            have_ui_data = true;
        }

        ui_print_state_t state = get_print_state();
        bool active = (state != UI_STATE_IDLE && state != UI_STATE_DONE && state != UI_STATE_FAULT);
        if (active && !timer_running) {
            timer_running = true;
            print_start = xTaskGetTickCount();
        } else if (!active && timer_running) {
            timer_running = false;   // freeze the last value on screen
        }
        if (timer_running) {
            elapsed_sec = (xTaskGetTickCount() - print_start) / configTICK_RATE_HZ;
        }

        if (!have_ui_data) continue;

        // Menus only change on a button event; the status screen redraws every period.
        if (!got_event && ui_data.screen != STATUS_SCREEN) continue;

        switch (ui_data.screen) {
            case MAIN_SCREEN:
                ui_draw_main_screen();
                oled_highlight_line(2 + ui_data.selection); // the +2 offset starts highlighting the third row
                break;
            case MENU_SCREEN:
                ui_draw_menu_screen();
                oled_highlight_line(2 + ui_data.selection);
                break;
            case STATUS_SCREEN: {
                ui_live_t live;
                fill_live_status(&live, state, elapsed_sec);
                ui_draw_status_screen(&live);
                break;
            }
            default:
                break;
        }

        oled_flush();
    }
}

void app_main(void) {
    // FIRST: drive heater/fan MOSFET gates to a defined 0% duty. Until this runs
    // the pins float, and a floating gate can partly switch a heater on.
    mosfet_driver_init();

    ESP_LOGI(TAG_MAIN, "Initializing IPC queues...");

    sys_event_group = xEventGroupCreate();

    motion_queue     = xQueueCreate(16, sizeof(PlannedMotion));
    gcode_cmds_queue = xQueueCreate(2, sizeof(GCodeCommand));
    gcode_line_queue = xQueueCreate(2, GCODE_LINE_MAX_LEN);
    thermal_cmds_queue = xQueueCreate(THERMAL_CMD_QUEUE_LEN, sizeof(thermal_cmd_t));
    ui_queue = xQueueCreate(5, sizeof(ui_data_t));
    gpio_evt_queue = xQueueCreate(10, sizeof(ButtonEdge_t));
    
    if (!motion_queue || !gcode_cmds_queue || !gcode_line_queue || !thermal_cmds_queue) {
        ESP_LOGE(TAG_MAIN, "Failed to allocate FreeRTOS Queues!");
        return;
    }

    ESP_LOGI(TAG_MAIN, "Spawning FreeRTOS tasks...");

    xTaskCreatePinnedToCore(parser_task, "Parser_Task", 4096, NULL, 2, &xParserTaskHandle, 0);
    xTaskCreatePinnedToCore(motion_planner_task, "Planner_Task", 4096, NULL, 2, &xPlannerTaskHandle, 0);
    xTaskCreatePinnedToCore(step_generator_task, "Step_Generator_Task", 4096, NULL, 2, &xStepGenTaskHandle, 1);
    xTaskCreatePinnedToCore(sd_streamer_task, "SD_Streamer_Task", 4096, NULL, 2, &xSDTaskHandle, 0);
    xTaskCreatePinnedToCore(sys_state_machine_task, "Sys_State_Machine_Task", 4096, NULL, 2, &xSysStateTaskHandle, 0);
    xTaskCreatePinnedToCore(thermal_task, "Thermal_Task", 4096, NULL, 3, &xThermalTaskHandle, 0);
    xTaskCreatePinnedToCore(ui_task, "UI_Task", 4096, NULL, 2, &xUITaskHandle, 0);

    ESP_LOGI(TAG_MAIN, "Initialization complete. Scheduler running.");
}

void print_motion_block(const PlannedMotion* block) {
    if (block == NULL) {
        ESP_LOGE(TAG_MOTION, "Cannot print NULL motion block.");
        return;
    }

    ESP_LOGI(TAG_MOTION, "=== Planned Motion Block Details ===");

    // Geometry & Distances
    ESP_LOGI(TAG_MOTION, "Path Length: %.2f mm | Total Vector Len: %.2f",
             block->path_length_mm, block->total_vector_length);
    ESP_LOGI(TAG_MOTION, "Unit Vector (X,Y,Z,E): (%.2f, %.2f, %.2f, %.2f)", 
             block->unit_vec[0], block->unit_vec[1], block->unit_vec[2], block->unit_vec[3]);

    // Velocities & Accelerations
    ESP_LOGI(TAG_MOTION, "Velocities - Entry: %.2f | Cruise: %.2f | Exit: %.2f", 
             block->v_entry, block->v_cruise, block->v_exit);
    ESP_LOGI(TAG_MOTION, "Max Accel - Path: %.2f | Vector: %.2f", 
             block->max_path_acceleration, block->max_vector_acceleration);

    // Axis Mapping & Steps
    ESP_LOGI(TAG_MOTION, "Master Axis: %d | Master Steps: %d  | Master Steps/mm: %.2f", 
             block->master_axis, block->master_steps, block->master_steps_per_mm);
    ESP_LOGI(TAG_MOTION, "Dir Bits: 0x%02X", block->dir_bits);
    ESP_LOGI(TAG_MOTION, "Axis Steps (X,Y,Z,E): (%d, %d, %d, %d)", block->steps[0], block->steps[1], 
            block->steps[2], block->steps[3]);

    // Trapezoidal Phase Step Counts
    ESP_LOGI(TAG_MOTION, "Phases - Accel Steps: %d | Cruise Steps: %d | Decel Steps: %d", 
            block->accel_steps, block->cruise_steps, block->decel_steps);
}