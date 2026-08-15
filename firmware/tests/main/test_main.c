#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "oled_test.h"
#include "i2c_oled.h"

void app_main(void) {
    oled_init();

    while(1) {
        // Turn on all pixels to verify power & panel status
        oled_test_fill(1);
    
        vTaskDelay(pdMS_TO_TICKS(3000)); // Stay on for 3 seconds
    
        // Return to normal RAM contents
        oled_test_fill(0);

        vTaskDelay(pdMS_TO_TICKS(3000)); // Stay off for 3 seconds
    }
}