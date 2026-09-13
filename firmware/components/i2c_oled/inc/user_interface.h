#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include <stdbool.h>
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

void ui_draw_menu_screen(void);
void ui_draw_status_screen(int nozzle_temp, int bed_temp, float z_position, int progress);
void ui_draw_main_screen(void);
bool ui_isEqual(ui_data_t *ui_data, ui_data_t *ui_data_old);
void handle_main_selection(int *selection, ui_data_t *ui_data);
void handle_menu_selection(int *selection, ui_data_t *ui_data);
void handle_status_screen_selection(int* selection ,ui_data_t *ui_data);

#endif