
#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define RED_LED_PIN GPIO_NUM_4 

static const char *TAG = "BLINK_TEST";

void app_main(void) {
    
    gpio_set_direction(RED_LED_PIN, GPIO_MODE_OUTPUT);
    
    int ledStatus = 0;
    while (1) {
        //ESP_LOGI(TAG, "Setting LED state to: %d", ledStatus);

        gpio_set_level(RED_LED_PIN, ledStatus);
        ledStatus = !ledStatus;

        vTaskDelay(pdMS_TO_TICKS(16)); // ~31Hz
    }
}
