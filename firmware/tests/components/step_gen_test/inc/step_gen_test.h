#ifndef STEP_GEN_TEST_H
#define STEP_GEN_TEST_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio_ll.h"

#define ENDSTOP_X_GPIO      GPIO_NUM_0  // these are for test only
#define ENDSTOP_Y_GPIO      GPIO_NUM_2
#define ENDSTOP_Z_GPIO      GPIO_NUM_4

#define STEP_PIN_X      GPIO_NUM_22
#define STEP_PIN_Y      GPIO_NUM_23
#define STEP_PIN_Z      GPIO_NUM_21

#define RMT_CHANNEL_X   0
#define RMT_CHANNEL_Y   1
#define RMT_CHANNEL_Z   2

// Structure bundling per-axis hardware configuration
typedef struct {
    uint8_t axis_id;        // 0 = X, 1 = Y, 2 = Z (Used for task bitmask)
    gpio_num_t step_pin;    // Step pin to decouple/force LOW
    uint8_t rmt_channel;    // RMT peripheral channel to force-reset
} axis_endstop_config_t;

void step_generator_task(void *pvParameters);
void endstops_test(void);
TaskHandle_t start_step_generator_task(void);
void init_endstops(void);

#endif