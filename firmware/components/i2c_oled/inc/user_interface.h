#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include <stdbool.h>
#include <stdint.h>
#include "sys_state_machine.h"

#define SYS_RUNNING_BIT (1 << 0)

typedef enum {
    MAIN_SCREEN,
    MENU_SCREEN,
    STATUS_SCREEN
} display_screen_t;

typedef struct {
    int status;
    int progress;
    int temp;

    int selection;
    display_screen_t screen;
    ButtonEvent_t button_event;
} ui_data_t;

typedef enum {
    UI_STATE_IDLE,
    UI_STATE_HEATING,
    UI_STATE_HOMING,
    UI_STATE_PRINTING,
    UI_STATE_PAUSED,
    UI_STATE_DONE,
    UI_STATE_FAULT
} ui_print_state_t;

// Live printer parameters for the status screen (filled by the UI task).
typedef struct {
    ui_print_state_t state;
    float nozzle_temp, nozzle_target;
    float bed_temp, bed_target;
    float x, y, z;
    int progress;           // 0..100, or -1 when no file is open
    uint32_t elapsed_sec;
} ui_live_t;

void ui_draw_menu_screen(void);
void ui_draw_status_screen(const ui_live_t *live);
void ui_draw_main_screen(void);
bool ui_isEqual(ui_data_t *ui_data, ui_data_t *ui_data_old);
void handle_main_selection(int *selection, ui_data_t *ui_data);
void handle_menu_selection(int *selection, ui_data_t *ui_data);
void handle_status_screen_selection(int* selection ,ui_data_t *ui_data);

#endif