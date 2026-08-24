#ifndef STEP_GEN_H
#define STEP_GEN_H

#include <stdint.h>
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "motion_planner.h"

#define X_STEP_PIN          GPIO_NUM_47
#define Y_STEP_PIN          GPIO_NUM_39
#define Z_STEP_PIN          GPIO_NUM_41
#define E_STEP_PIN          GPIO_NUM_2

#define X_DIR_PIN           GPIO_NUM_21
#define Y_DIR_PIN           GPIO_NUM_38
#define Z_DIR_PIN           GPIO_NUM_40
#define E_DIR_PIN           GPIO_NUM_42

#define NUM_AXES            4
#define SYMBOLS_PER_BLOCK   64
#define MEM_BLOCKS_PER_AXIS 2  // 2 blocks = 128 symbols total per channel for Ping-Pong

// Struct holding handles for all 4 printer axes
typedef struct {
    rmt_channel_handle_t tx_channels[NUM_AXES];
    rmt_encoder_handle_t copy_encoders[NUM_AXES];
} rmt_stepper_system_t;

typedef struct {
    uint32_t accumulators[NUM_AXES];
    uint32_t master_step_count;
    int32_t  c;    // Current Master step delay (RMT ticks)
    int32_t  rest; // Precision remainder
    
    PlannedMotion block;
} multi_axis_dda_generator_t;

void register_stepper_callbacks(rmt_stepper_system_t *sys, TaskHandle_t generator_task);
void init_stepper_rmt_channels(rmt_stepper_system_t *sys, const uint8_t gpio_pins[NUM_AXES]);
void generate_dda_rmt_buffers(multi_axis_dda_generator_t *dda, 
    rmt_symbol_word_t buffers[NUM_AXES][SYMBOLS_PER_BLOCK], 
    size_t *generated_symbols
);


#endif