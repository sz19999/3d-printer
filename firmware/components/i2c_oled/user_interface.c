#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "user_interface.h"
#include "i2c_oled.h"

extern EventGroupHandle_t sys_event_group;
extern TaskHandle_t xStepGenTaskHandle;

extern QueueHandle_t ui_queue;
extern QueueHandle_t gpio_evt_queue;
extern QueueHandle_t gcode_line_queue;
extern QueueHandle_t gcode_cmds_queue;
extern QueueHandle_t motion_queue;
extern QueueHandle_t thermal_cmds_queue;


static const char *ui_state_name(ui_print_state_t state) {
    switch (state) {
        case UI_STATE_HEATING:  return "HEATING";
        case UI_STATE_HOMING:   return "HOMING";
        case UI_STATE_PRINTING: return "PRINTING";
        case UI_STATE_PAUSED:   return "PAUSED";
        case UI_STATE_DONE:     return "DONE";
        case UI_STATE_FAULT:    return "THERMAL FAULT";
        case UI_STATE_IDLE:
        default:                return "IDLE";
    }
}

// Outline + fill of a horizontal progress bar (pixel rows y0..y0+h-1).
static void ui_draw_progress_bar(int32_t y0, int32_t h, int percent) {
    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;
    int32_t fill = ((OLED_WIDTH - 4) * percent) / 100;

    for (int32_t x = 0; x < OLED_WIDTH; x++) {
        oled_draw_pixel(x, y0, 1);
        oled_draw_pixel(x, y0 + h - 1, 1);
    }
    for (int32_t y = y0; y < y0 + h; y++) {
        oled_draw_pixel(0, y, 1);
        oled_draw_pixel(OLED_WIDTH - 1, y, 1);
    }
    for (int32_t y = y0 + 2; y < y0 + h - 2; y++) {
        for (int32_t x = 2; x < 2 + fill; x++) {
            oled_draw_pixel(x, y, 1);
        }
    }
}

/*
    128x64 layout, 8 text lines of 21 chars:
    0  PRINTING       12:34
    1  Noz 215/220C
    2  Bed  58/ 60C
    3  X 102.3  Y  88.1
    4  Z  0.28mm   42%
    5  [=========        ]   (progress bar, pixel rows 41..47)
    7  Hold To Exit
*/
void ui_draw_status_screen(const ui_live_t *live) {
    char buffer[32];

    oled_clear_screen();

    uint32_t mins = live->elapsed_sec / 60;
    uint32_t secs = live->elapsed_sec % 60;
    snprintf(buffer, sizeof(buffer), "%-13s%3lu:%02lu", ui_state_name(live->state),
             (unsigned long)(mins > 999 ? 999 : mins), (unsigned long)secs);
    oled_print_line(0, buffer);

    snprintf(buffer, sizeof(buffer), "Noz %3.0f/%3.0fC", live->nozzle_temp, live->nozzle_target);
    oled_print_line(1, buffer);

    snprintf(buffer, sizeof(buffer), "Bed %3.0f/%3.0fC", live->bed_temp, live->bed_target);
    oled_print_line(2, buffer);

    snprintf(buffer, sizeof(buffer), "X%6.1f  Y%6.1f", live->x, live->y);
    oled_print_line(3, buffer);

    if (live->progress >= 0) {
        snprintf(buffer, sizeof(buffer), "Z%6.2fmm    %3d%%", live->z, live->progress);
    } else {
        snprintf(buffer, sizeof(buffer), "Z%6.2fmm", live->z);
    }
    oled_print_line(4, buffer);

    ui_draw_progress_bar(41, 7, live->progress >= 0 ? live->progress : 0);

    oled_print_line(7, "Hold To Exit");
}

void ui_draw_menu_screen(void) {
    oled_clear_screen();
    
    oled_print_line(0, "--- MENU SCREEN ---");
    oled_print_line(2, " Start Print");
    oled_print_line(3, " Pause Print");
    oled_print_line(4, " Abort Print");
    oled_print_line(5, " Back");
}

void ui_draw_main_screen(void) {
    oled_clear_screen();
    
    oled_print_line(0, "--- MAIN SCREEN ---");
    oled_print_line(2, " Menu");
    oled_print_line(3, " Status");
    oled_print_line(5, "Single - Down");
    oled_print_line(6, "Double - Up");
    oled_print_line(7, "Hold   - Select");
}

bool ui_isEqual(ui_data_t *ui_data, ui_data_t *ui_data_old) {
    return (ui_data->selection == ui_data_old->selection) &&
           (ui_data->screen == ui_data_old->screen) &&
           (ui_data->button_event == ui_data_old->button_event);
}

void handle_main_selection(int *selection, ui_data_t *ui_data) {
    switch(*selection) {
        // Handlers only change ui_data->screen; the UI task is the only one that
        // draws, so two tasks never write the shared OLED buffer at once.
        case 0: // Menu
            ui_data->screen = MENU_SCREEN;
            break;
        case 1: // Status
            ui_data->screen = STATUS_SCREEN;
            break;
        default:
            break;
    }
    *selection = 0;
}

void handle_menu_selection(int *selection, ui_data_t *ui_data) {
    switch(*selection) {
        case 0: // Start Print
            // wake up all tasks, this starts the printing sequence
            xEventGroupSetBits(sys_event_group, SYS_RUNNING_BIT);
            vTaskResume(xStepGenTaskHandle);
            ui_data->screen = STATUS_SCREEN;
            break;
        case 1: // Pause Print
            // halt step generator task to stop printing
            // keep print head warm, to avoid re-heating
            vTaskSuspend(xStepGenTaskHandle);
            break;
        case 2: // Abort Print
            // halt all tasks, flush all queues
            xEventGroupClearBits(sys_event_group, SYS_RUNNING_BIT);
            vTaskSuspend(xStepGenTaskHandle);
            xQueueReset(gcode_line_queue);
            xQueueReset(gcode_cmds_queue);
            xQueueReset(motion_queue);
            xQueueReset(thermal_cmds_queue);
            break;
        case 3: // Back
            // show main screen
            ui_data->screen = MAIN_SCREEN;
            break;
        default:
            break;
    }
    *selection = 0;
}

void handle_status_screen_selection(int* selection, ui_data_t *ui_data) {
    ui_data->screen = MAIN_SCREEN;
    *selection = 0;
}