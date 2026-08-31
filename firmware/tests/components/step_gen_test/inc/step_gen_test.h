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


void step_generator_task(void *pvParameters);
void endstops_test(void);

#endif