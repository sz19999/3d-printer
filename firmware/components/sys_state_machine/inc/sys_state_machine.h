#ifndef SYS_STATE_MACHINE_H
#define SYS_STATE_MACHINE_H

#include <stdbool.h>

#define NUM_MENU_ITEMS   3           // 3 options in menu
#define MENU_TIMEOUT 1000000 * 5 // 5sec
#define BTN_PIN      GPIO_NUM_36    // 48 is the original pin

typedef enum {
    EVENT_NONE,
    EVENT_SINGLE_CLICK,
    EVENT_DOUBLE_CLICK,
    EVENT_LONG_PRESS
} ButtonEvent_t;

typedef struct {
    bool is_pressed;      // true = pressed, false = released
    int64_t timestamp_us; // Exact time captured by hardware
} ButtonEdge_t;

ButtonEvent_t process_button_edges(void);
void init_button_interrupt(void);

#endif