#include "step_generator.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "motion_planner.h"
#include "config.h"
#include "driver/gpio.h"
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include "esp_attr.h"
#include "esp_log.h"

static const char *TAG_RMT = "RMT_STEPPER";

void init_stepper_rmt_channels(rmt_stepper_system_t *sys, const uint8_t step_pins[NUM_AXES]) {
    rmt_copy_encoder_config_t encoder_config = {};

    
    for (int i = 0; i < NUM_AXES; i++) {
        gpio_reset_pin(step_pins[i]);
        gpio_set_direction(step_pins[i], GPIO_MODE_OUTPUT);

        rmt_tx_channel_config_t tx_chan_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .gpio_num = step_pins[i],
            .resolution_hz = 1000000,               // 1 MHz = 1 tick per microsecond
            .mem_block_symbols = 48, // 64 symbols = 1 HW block
            .trans_queue_depth = 4,                 // Software depth for DRAM ping-ponging
            .flags = {
                .with_dma = false,
            }
        };

        esp_err_t ret = rmt_new_tx_channel(&tx_chan_config, &sys->tx_channels[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_RMT, "Axis %d registration failed! Error: %s (0x%x)", i, esp_err_to_name(ret), ret);
            return;
        }
        
        ESP_ERROR_CHECK(rmt_new_copy_encoder(&encoder_config, &sys->copy_encoders[i]));
        ESP_ERROR_CHECK(rmt_enable(sys->tx_channels[i]));
        ESP_LOGI(TAG_RMT, "Axis %d initialized successfully on GPIO %d", i, step_pins[i]);
    }
}


// callback triggered in ISR context when RMT finishes sending a buffer block
static bool IRAM_ATTR rmt_stepper_done_cb(rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx) {
    BaseType_t high_task_woken = pdFALSE;
    TaskHandle_t generator_task_handle = (TaskHandle_t)user_ctx;

    // signal the step generator task to prepare the next ping-pong buffer
    vTaskNotifyGiveFromISR(generator_task_handle, &high_task_woken);

    return high_task_woken == pdTRUE;
}

void register_stepper_callbacks(rmt_stepper_system_t *sys, TaskHandle_t generator_task) {
    rmt_tx_event_callbacks_t cbs = {
        .on_trans_done = rmt_stepper_done_cb,
    };
    
    // attach the callback to one of the axes (e.g. X axis)
    rmt_tx_register_event_callbacks(sys->tx_channels[0], &cbs, (void *)generator_task);
}

void generate_dda_rmt_buffers(multi_axis_dda_generator_t *dda, 
    rmt_symbol_word_t buffers[NUM_AXES][SYMBOLS_PER_BLOCK], 
    size_t *generated_symbols
) {
    size_t symbol_idx = 0;
    PlannedMotion *b = &dda->block;

    while (symbol_idx < SYMBOLS_PER_BLOCK && dda->master_step_count < b->master_steps) {
        uint32_t n = dda->master_step_count;

        // 1. Compute Master Axis Step Delay via David Austin
        if (n == 0) {
            float dS = 0;
            if (b->master_axis == 0 || b->master_axis == 1) {
                dS = 1.0f / STEPS_PER_MM_BELT;
            }
            else if (b->master_axis == 2) {
                dS = 1.0f / STEPS_PER_MM_SCREW;
            }
            else if (b->master_axis == 3) {
                dS = 1.0f / STEPS_PER_MM_GEAR;
            }


            float c0_sec = sqrtf((2.0f * dS) / b->max_path_acceleration);
            dda->c = lroundf(c0_sec * 1000000.0f);  // convert seconds to microseconds
            dda->rest = 0;
        } else if (n < b->accel_steps) {
            int32_t top = (2 * dda->c) + dda->rest;
            int32_t div = (4 * n) + 1;
            dda->c -= top / div;
            dda->rest = top % div;
        } else if (n >= b->accel_steps + b->cruise_steps) {
            uint32_t m = b->master_steps - n;
            int32_t top = (2 * dda->c) + dda->rest;
            int32_t div = (4 * m) + 1;
            dda->c += top / div;
            dda->rest = top % div;
        }

        uint16_t pulse_ticks = 2; // 2us HIGH pulse width
        uint16_t low_ticks = (dda->c > pulse_ticks) ? (dda->c - pulse_ticks) : 1;

        for (int axis = 0; axis < NUM_AXES; axis++) {
            bool send_step = false;

            if (axis == b->master_axis) {
                send_step = true;
            } else {
                dda->accumulators[axis] += b->steps[axis];
                if (dda->accumulators[axis] >= b->master_steps) {
                    send_step = true;
                    dda->accumulators[axis] -= b->master_steps;
                }
            }

            if (send_step) {
                buffers[axis][symbol_idx] = (rmt_symbol_word_t){
                    .duration0 = pulse_ticks,
                    .level0 = 1,
                    .duration1 = low_ticks,
                    .level1 = 0
                };
            } else {
                buffers[axis][symbol_idx] = (rmt_symbol_word_t){
                    .duration0 = dda->c,
                    .level0 = 0,
                    .duration1 = 0,
                    .level1 = 0
                };
            }
        }
        dda->master_step_count++;
        symbol_idx++;
    }

    *generated_symbols = symbol_idx;
}