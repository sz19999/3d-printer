#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "gcode_parser.h"
#include "motion_planner.h"

#define GCODE_LINE_MAX_LEN 128

static const char *TAG_PARSER  = "PARSER_TASK";
static const char *TAG_PLANNER = "PLANNER_TASK";
static const char *TAG_MAIN    = "MAIN_APP";

// Global handles for queues
static QueueHandle_t gcode_line_queue = NULL;
static QueueHandle_t gcode_cmds_queue = NULL;
static QueueHandle_t motion_queue = NULL;


void parser_task(void *pvParameters) {
    char gcode_line[GCODE_LINE_MAX_LEN];
    GCodeCommand gcode_cmd;

    ESP_LOGI(TAG_PARSER, "Task started successfully on core %d", xPortGetCoreID());

    while (1) {
        // Wait indefinitely for raw gcode string lines
        if (xQueueReceive(gcode_line_queue, gcode_line, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG_PARSER, "Received raw line: \"%s\"", gcode_line);

            // Parse raw gcode line text buffer
            if (!parse_command(gcode_line, &gcode_cmd)) {
                ESP_LOGE(TAG_PARSER, "Failed to parse command: \"%s\"", gcode_line);
                continue;
            }

            ESP_LOGI(TAG_PARSER, "Successfully parsed G-Code command type/code.");

            // Dispatch into the parsed gcode queue
            if (xQueueSend(gcode_cmds_queue, &gcode_cmd, pdMS_TO_TICKS(100)) == pdPASS) {
                ESP_LOGI(TAG_PARSER, "Dispatched GCodeCommand to gcode_cmds_queue.");
            } else {
                ESP_LOGE(TAG_PARSER, "gcode_cmds_queue full! Command dropped.");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Brief delay for smooth serial observation
    }
}

void motion_planner_task(void *pvParameters) {
    GCodeCommand gcode_cmd;
    MotionMetadata metadata;
    PlannedMotion* motion = NULL;
    RingBuffer buffer;

    PointMM current_mm = {0.0f, 0.0f, 0.0f, 0.0f};
    PointSteps current_steps = {0, 0, 0, 0};
    bool absolute_mode = true;

    init_buffer(&buffer);
    ESP_LOGI(TAG_PLANNER, "Task started successfully on core %d", xPortGetCoreID());
    
    while (1) {
        if (xQueueReceive(gcode_cmds_queue, &gcode_cmd, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG_PLANNER, "Received parsed command from queue.");

            if (is_motion_command(&gcode_cmd)) {
                ESP_LOGI(TAG_PLANNER, "Command identified as MOTION. Planning velocity profile...");

                handle_motion_command(&gcode_cmd, &buffer, &current_mm, &current_steps, &absolute_mode);
                motion = front(&buffer);

                // dispatch oldest motion to the step generator
                if (motion != NULL) {
                    ESP_LOGI(TAG_PLANNER, "Buffer front returned a motion block.");
                    if (xQueueSend(motion_queue, motion, pdMS_TO_TICKS(100)) == pdPASS) {
                        ESP_LOGI(TAG_PLANNER, "Dispatched PlannedMotion block to motion_queue.");
                    } else {
                        ESP_LOGE(TAG_PLANNER, "motion_queue full! Could not send motion block.");
                    }
                    pop(&buffer);
                } else {
                    ESP_LOGW(TAG_PLANNER, "Buffer front returned NULL after handling motion command.");
                }
            } else {
                ESP_LOGI(TAG_PLANNER, "Command identified as NON-MOTION (Heater/Fan/State). Processing metadata...");
                extract_metadata(&gcode_cmd, &metadata);
                // System state handling logic goes here
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    ESP_LOGI(TAG_MAIN, "Initializing IPC queues...");

    motion_queue     = xQueueCreate(10, sizeof(PlannedMotion));
    gcode_cmds_queue = xQueueCreate(10, sizeof(GCodeCommand));
    gcode_line_queue = xQueueCreate(10, GCODE_LINE_MAX_LEN);

    if (!motion_queue || !gcode_cmds_queue || !gcode_line_queue) {
        ESP_LOGE(TAG_MAIN, "Failed to allocate FreeRTOS Queues!");
        return;
    }

    // Pre-fill line queue with test G-code strings
    const char *gcode_commands[10] = {
        "G28 ; Home all axes",
        "G90 ; Set positioning to absolute",
        "M104 S200 ; Set extruder temperature to 200C",
        "M140 S60 ; Set bed temperature to 60C",
        "M109 S200 ; Wait for extruder temperature",
        "M190 S60 ; Wait for bed temperature",
        "G1 Z0.2 F1200 ; Move nozzle to initial layer height",
        "G1 X10 Y10 E1.0 F1500 ; Extrude a small line segment",
        "G1 X50 Y10 E3.5 F1500 ; Continue printing motion path",
        "M107 ; Turn off fan"
    };

    ESP_LOGI(TAG_MAIN, "Pre-filling gcode_line_queue with 10 test strings...");
    for (int i = 0; i < 10; i++) {
        if (xQueueSend(gcode_line_queue, gcode_commands[i], pdMS_TO_TICKS(100)) == pdPASS) {
            ESP_LOGI(TAG_MAIN, "Pushed string [%d/10] into line queue.", i + 1);
        } else {
            ESP_LOGE(TAG_MAIN, "Line queue full when pushing string index %d", i);
        }
    }

    ESP_LOGI(TAG_MAIN, "Spawning FreeRTOS tasks...");

    xTaskCreatePinnedToCore(
        parser_task,
        "Parser_Task",
        4096,
        NULL,
        2,
        NULL,
        0
    );

    xTaskCreatePinnedToCore(
        motion_planner_task,
        "Planner_Task",
        4096,
        NULL,
        2,
        NULL,
        0
    );

    ESP_LOGI(TAG_MAIN, "Initialization complete. Scheduler running.");
}