#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "oled_test.h"
#include "i2c_oled.h"

void draw_pixel_by_pixel(void) {
    oled_init();

    for (int j = 0; j < OLED_HEIGHT; j++) {
        for (int i = 0; i < OLED_WIDTH; i++) {
            oled_draw_pixel(i, j, 1);
            oled_flush();
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

void print_lines(void) {
    oled_init();
    oled_print_line(0, "stop");
    oled_print_line(1, "1");
    oled_print_line(2, "2");
    oled_print_line(3, "3");
    oled_print_line(4, "continue");
    oled_highlight_line(4);
    oled_print_line(5, "5");
    oled_print_line(6, "6");
    oled_print_line(7, "status: waiting");
    oled_flush();
}