#ifndef STEP_GEN_H
#define STEP_GEN_H

#include <stdint.h>
#include "motion_planner.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "motion_planner.h"
#include "driver/gpio.h"
#include "hal/gpio_ll.h"
#include "soc/rmt_struct.h"

#define X_STEP_PIN          GPIO_NUM_16 // 47 (the commented pins are the original pins)      
#define Y_STEP_PIN          GPIO_NUM_17 // 38
#define Z_STEP_PIN          GPIO_NUM_5  // 40    
#define E_STEP_PIN          GPIO_NUM_21 // 42

#define X_DIR_PIN           GPIO_NUM_21 
#define Y_DIR_PIN           GPIO_NUM_32 // 39
#define Z_DIR_PIN           GPIO_NUM_31 // 41
#define E_DIR_PIN           GPIO_NUM_2

#define ENDSTOP_X_GPIO      GPIO_NUM_0  // these are for test only
#define ENDSTOP_Y_GPIO      GPIO_NUM_2
#define ENDSTOP_Z_GPIO      GPIO_NUM_4

#define RMT_CHANNEL_X       0
#define RMT_CHANNEL_Y       1
#define RMT_CHANNEL_Z       2
#define RMT_CHANNEL_E       3

#define NUM_AXES            4
#define SYMBOLS_PER_BLOCK   48 // 64
#define MEM_BLOCKS_PER_AXIS 2  // 2 blocks = 128 symbols total per channel for Ping-Pong

// Endstop Abort Bits (Bits 0–3)
#define ENDSTOP_X_TRIGGERED  (1 << 0)  // 0x01
#define ENDSTOP_Y_TRIGGERED  (1 << 1)  // 0x02
#define ENDSTOP_Z_TRIGGERED  (1 << 2)  // 0x04
#define ENDSTOP_E_TRIGGERED  (1 << 3)  // 0x08
#define ENDSTOP_ABORT_MASK   (ENDSTOP_X_TRIGGERED | ENDSTOP_Y_TRIGGERED | \
                              ENDSTOP_Z_TRIGGERED | ENDSTOP_E_TRIGGERED) // 0x0F

// RMT Driver Bits (Bit 31)
#define RMT_TX_DONE_BIT      (1 << 31) // 0x80000000

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

typedef struct {
    uint8_t axis_id;        // 0 = X, 1 = Y, 2 = Z (Used for task bitmask)
    gpio_num_t step_pin;    // Step pin to decouple/force LOW
    uint8_t rmt_channel;    // RMT peripheral channel to force-reset
} axis_endstop_config_t;

void register_stepper_callbacks(rmt_stepper_system_t *sys, TaskHandle_t generator_task);
void init_stepper_rmt_channels(rmt_stepper_system_t *sys, const uint8_t gpio_pins[NUM_AXES]);
void generate_dda_rmt_buffers(multi_axis_dda_generator_t *dda, 
    rmt_symbol_word_t buffers[NUM_AXES][SYMBOLS_PER_BLOCK], 
    size_t *generated_symbols
);
void init_endstops(void);


#endif