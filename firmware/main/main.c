#include <stdio.h>
#include <dirent.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "gcode_parser.h"
#include "motion_planner.h"
#include "spi_sd.h"
#include "step_generator.h"
#include "sys_state_machine.h"

#define GCODE_LINE_MAX_LEN 128
#define SYS_RUNNING_BIT (1 << 0)

const char *TAG_PARSER  = "PARSER_TASK";
const char *TAG_PLANNER = "PLANNER_TASK";
const char *TAG_MAIN    = "MAIN_APP";
const char *TAG_MOTION  = "MOTION_BLOCK_TASK";
const char *TAG_SD      = "SD_STREAMER_TASK";
const char *TAG_RMT     = "STEP_GENERATOR_TASK";
const char *TAG_SYS     = "SYS_STATE_TASK";

// Global handles for queues
QueueHandle_t gcode_line_queue = NULL;
QueueHandle_t gcode_cmds_queue = NULL;
QueueHandle_t motion_queue = NULL;
QueueHandle_t gpio_evt_queue = NULL;

TaskHandle_t xParserTaskHandle  = NULL;
TaskHandle_t xSDTaskHandle      = NULL;
TaskHandle_t xPlannerTaskHandle = NULL;
TaskHandle_t xStepGenTaskHandle = NULL;
TaskHandle_t xSysStateTaskHandle = NULL;

EventGroupHandle_t sys_event_group;

motion_mode_t current_motion_mode;

// shared between motion planner and PID controller
MotionMetadata metadata;


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
                fclose(f);
                vTaskDelay(pdMS_TO_TICKS(1000000)); 
                continue;
            }

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

void motion_planner_task(void *pvParameters) {
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
                handle_metadata_command(&gcode_cmd, &metadata);
                // TODO: dispatch printing status to OLED Task
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
                
                if (xQueueSend(motion_queue, motion, pdMS_TO_TICKS(10)) == pdPASS) {
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

    // initialize RMT Hardware Channels
    init_stepper_rmt_channels(&sys, step_pins);

    // register callback only on Master Channel
    register_stepper_callbacks(&sys, xTaskGetCurrentTaskHandle());
    
    // configure direction pins as outputs
    for (int i = 0; i < NUM_AXES; i++) {
        gpio_reset_pin(dir_pins[i]);
        gpio_set_direction(dir_pins[i], GPIO_MODE_OUTPUT);
    }

    while (1) {
        xEventGroupWaitBits(
                sys_event_group,
                SYS_RUNNING_BIT,
                pdFALSE,        // Don't clear bit on exit (so other tasks stay unblocked)
                pdTRUE,         // Wait for all bits
                portMAX_DELAY   // Wait indefinitely
            );

        // wait for next motion block from motion planner queue
        if (xQueueReceive(motion_queue, &motion, portMAX_DELAY) == pdTRUE) {
            bool abort_triggered = false;
            uint32_t notify_value = 0;

            ESP_LOGI(TAG_RMT, "Received motion block -> master_steps: %lu, dir_bits: 0x%02X", 
                     (unsigned long)motion.master_steps, motion.dir_bits);

            // set direction pins
            for (int i = 0; i < NUM_AXES; i++) {
                gpio_set_level(dir_pins[i], (motion.dir_bits >> i) & 0x01);
            }

            // initialize DDA state machine for new move block
            dda.block = motion;
            dda.master_step_count = 0;
            memset(dda.accumulators, 0, sizeof(dda.accumulators));

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
                        abort_triggered = true;
                        break; // exit streaming loop immediately
                    }

                    // otherwise, notification is RMT TX_DONE callback
                    active_transports--;

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
                    }
                }
            } 
            

            if (abort_triggered) {
                if (current_motion_mode == MOTION_MODE_HOMING) {
                    // HOMING MODE: Stop only the tripped axis
                    uint8_t tripped_axis = (notify_value >> 1) & 0x03;
                    
                    ESP_LOGI(TAG_RMT, "Homing touch detected on Axis %d", tripped_axis);
                    rmt_disable(sys.tx_channels[tripped_axis]);
                    rmt_enable(sys.tx_channels[tripped_axis]); // Re-arm immediately for back-off move
                    
                    // Motion queue is NOT flushed so back-off moves continue cleanly

                } else {
                    // PRINTING MODE: Unexpected Crash / Hard Limit
                    ESP_LOGE(TAG_RMT, "CRITICAL: Unexpected endstop trip during print execution!");
                    
                    for (int axis = 0; axis < NUM_AXES; axis++) {
                        rmt_disable(sys.tx_channels[axis]);
                        rmt_enable(sys.tx_channels[axis]);
                    }
                    
                    xQueueReset(motion_queue); // Flush remaining G-code execution stream
                }
            }

            ESP_LOGI(TAG_RMT, "Motion block execution finished (%u ping-pong refills)", (unsigned int)iterations);
        }
    }
}

void sys_state_machine_task(void *pvParameters) {
    ESP_LOGI(TAG_SYS, "Task started successfully on core %d", xPortGetCoreID());

    // Broadcast ready signal to all unblocked tasks simultaneously
    xEventGroupSetBits(sys_event_group, SYS_RUNNING_BIT);

    init_button_interrupt();
    int selection = 0;

    while(1) {
        ButtonEvent_t event = process_button_edges();

        switch (event) {
            case EVENT_SINGLE_CLICK:
                selection = (selection + 1) % NUM_MENU_ITEMS;
                //update_display(selection);
                ESP_LOGI(TAG_SYS, "Single Click!");
                break;

            case EVENT_DOUBLE_CLICK:
                selection = (selection - 1 + NUM_MENU_ITEMS) % NUM_MENU_ITEMS;
                //update_display(selection);
                ESP_LOGI(TAG_SYS, "Double Click!");
                break;

            case EVENT_LONG_PRESS:
                //if (menu_items[selection].action) {
                    //menu_items[selection].action(); // Execute function pointer
                //}
                ESP_LOGI(TAG_SYS, "Long Press!");
                break;

            case EVENT_NONE:
            default:
                break;
        }

        vTaskDelay(1); // Cooperative yield for parallel tasks
    }
}

void app_main(void) {
    ESP_LOGI(TAG_MAIN, "Initializing IPC queues...");

    sys_event_group = xEventGroupCreate();

    motion_queue     = xQueueCreate(16, sizeof(PlannedMotion));
    gcode_cmds_queue = xQueueCreate(2, sizeof(GCodeCommand));
    gcode_line_queue = xQueueCreate(2, GCODE_LINE_MAX_LEN);
    
    if (!motion_queue || !gcode_cmds_queue || !gcode_line_queue) {
        ESP_LOGE(TAG_MAIN, "Failed to allocate FreeRTOS Queues!");
        return;
    }

    ESP_LOGI(TAG_MAIN, "Spawning FreeRTOS tasks...");

    xTaskCreatePinnedToCore(parser_task, "Parser_Task", 4096, NULL, 2, &xParserTaskHandle, 0);
    xTaskCreatePinnedToCore(motion_planner_task, "Planner_Task", 4096, NULL, 2, &xPlannerTaskHandle, 0);
    xTaskCreatePinnedToCore(step_generator_task, "Step_Generator_Task", 4096, NULL, 2, &xStepGenTaskHandle, 1);
    xTaskCreatePinnedToCore(sd_streamer_task, "SD_Streamer_Task", 4096, NULL, 2, &xSDTaskHandle, 0);
    xTaskCreatePinnedToCore(sys_state_machine_task, "Sys_State_Machine_Task", 4096, NULL, 2, &xSysStateTaskHandle, 0);

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