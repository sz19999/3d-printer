#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED_PIN   GPIO_NUM_4
#define STEP_PIN  GPIO_NUM_2
#define DIR_PIN   GPIO_NUM_1

void app_main(void)
{
    gpio_set_direction(STEP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(DIR_PIN, GPIO_MODE_OUTPUT);
    
    // turn on MCU power status LED
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 1); 

    int steps = 0;
    int dir = 1;
    while (1) {
        gpio_set_level(STEP_PIN, 0);
        esp_rom_delay_us(1000); // wait 1ms
        
        gpio_set_level(STEP_PIN, 1);
        esp_rom_delay_us(1000); // wait 1ms

        steps++;

        if (steps % 1600 == 0) {
            dir = !dir;
            gpio_set_level(DIR_PIN, dir);
        }
    }
}