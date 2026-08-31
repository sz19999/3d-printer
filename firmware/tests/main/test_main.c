
#include "sd_test.h"
#include "oled_test.h"
#include "tmc2209_test.h"
#include "step_gen_test.h"

void app_main(void) {
    //print_lines();    // oled_test
    //read_line();      // sd_test
    //spin_motor();        // tmc2209_test
    endstops_test();
}