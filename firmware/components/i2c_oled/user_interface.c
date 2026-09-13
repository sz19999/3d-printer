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


void ui_draw_status_screen(int nozzle_temp, int bed_temp, float z_position, int progress) {
    char buffer[30];
    
    oled_clear_screen();

    oled_print_line(0, "--- STATUS SCREEN ---");

    sprintf(buffer, "Nozzle: %d C", nozzle_temp);
    oled_print_line(2, buffer);

    sprintf(buffer, "Bed   : %d C", bed_temp);
    oled_print_line(3, buffer);

    sprintf(buffer, "Z-Axis: %.2f mm" , z_position);
    oled_print_line(4, buffer);

    sprintf(buffer, "Progress: %d%%", progress);
    oled_print_line(5, buffer);

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
        case 0: // Menu
            ui_draw_menu_screen();
            ui_data->screen = MENU_SCREEN;
            break;
        case 1: // Status
            ui_draw_status_screen(25, 25, 0, 0);
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
            ui_draw_status_screen(25, 25, 0, 0);
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
            ui_draw_main_screen();
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