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
#include "hal/rmt_ll.h"


extern const char *TAG_RMT;

// Per-axis configuration instances
static const axis_endstop_config_t AXIS_X_CFG = { .axis_id = 0, .step_pin = X_STEP_PIN, .rmt_channel = RMT_CHANNEL_X };
static const axis_endstop_config_t AXIS_Y_CFG = { .axis_id = 1, .step_pin = Y_STEP_PIN, .rmt_channel = RMT_CHANNEL_Y };
static const axis_endstop_config_t AXIS_Z_CFG = { .axis_id = 2, .step_pin = Z_STEP_PIN, .rmt_channel = RMT_CHANNEL_Z };

extern TaskHandle_t xStepGenTaskHandle;

static void IRAM_ATTR multi_axis_endstop_isr(void *arg) {
    // Cast void argument back to specific axis context
    const axis_endstop_config_t *axis = (const axis_endstop_config_t *)arg;

    // 1. HARDWARE OVERRIDE: Immediately force step pin LOW via Low-Layer HAL
    gpio_ll_set_level(&GPIO, axis->step_pin, 0);
    gpio_ll_output_enable(&GPIO, axis->step_pin);

    // 2. PERIPHERAL SHUTDOWN: Reset RMT hardware registers directly 
    // --- replace these to match for ESP32-S3 ---
    //RMT.conf_ch[axis->rmt_channel].conf1.tx_start   = 0; // Stop transmission
    //RMT.conf_ch[axis->rmt_channel].conf1.mem_rd_rst = 1; // Pulse read pointer reset high
    //RMT.conf_ch[axis->rmt_channel].conf1.mem_rd_rst = 0; // Release reset

    // 2. PERIPHERAL SHUTDOWN: Reset RMT hardware registers directly (ESP32-S3 compatible)
    rmt_ll_tx_stop(&RMT, axis->rmt_channel);            // Instantly stop TX hardware engine
    rmt_ll_tx_reset_pointer(&RMT, axis->rmt_channel);   // Reset memory read pointer to index 0

    // 3. SOFTWARE NOTIFICATION: Unblock Step Generator Task via bitmask
    if (xStepGenTaskHandle != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // Set bit corresponding to axis ID (Bit 0 = X, Bit 1 = Y, Bit 2 = Z)
        xTaskNotifyFromISR(
            xStepGenTaskHandle,
            (1 << axis->axis_id),
            eSetBits,
            &xHigherPriorityTaskWoken
        );

        // Yield immediately if Step Task has higher priority than current execution context
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// callback triggered in ISR context when RMT finishes sending a buffer block
static bool IRAM_ATTR rmt_stepper_done_cb(rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx) {
    BaseType_t high_task_woken = pdFALSE;
    TaskHandle_t generator_task_handle = (TaskHandle_t)user_ctx;

    // Use xTaskNotifyFromISR with eSetBits to pass specific event flags
    xTaskNotifyFromISR(
        generator_task_handle,
        RMT_TX_DONE_BIT,
        eSetBits,
        &high_task_woken
    );

    return high_task_woken == pdTRUE;
}


void init_stepper_rmt_channels(rmt_stepper_system_t *sys, const uint8_t step_pins[NUM_AXES]) {
    rmt_copy_encoder_config_t encoder_config = {};

    
    for (int i = 0; i < NUM_AXES; i++) {
        gpio_reset_pin(step_pins[i]);
        gpio_set_direction(step_pins[i], GPIO_MODE_OUTPUT);

        rmt_tx_channel_config_t tx_chan_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .gpio_num = step_pins[i],
            .resolution_hz = 1000000,               // 1 MHz = 1 tick per microsecond
            .mem_block_symbols = 64,                // 64 symbols = 1 HW block
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
                // RMT treats a duration of 0 as "hold this level forever", not "zero ticks" -
                // split the idle period into two nonzero halves so the channel actually finishes.
                uint16_t idle_half1 = (dda->c > 1) ? (uint16_t)(dda->c / 2) : 1;
                uint16_t idle_half2 = (dda->c > idle_half1) ? (uint16_t)(dda->c - idle_half1) : 1;

                buffers[axis][symbol_idx] = (rmt_symbol_word_t){
                    .duration0 = idle_half1,
                    .level0 = 0,
                    .duration1 = idle_half2,
                    .level1 = 0
                };
            }
        }
        dda->master_step_count++;
        symbol_idx++;
    }

    *generated_symbols = symbol_idx;
}


void init_endstops(void) {

    // Configure inputs with pull-ups (Active-LOW switches)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ENDSTOP_X_GPIO) | 
                        (1ULL << ENDSTOP_Y_GPIO) | 
                        (1ULL << ENDSTOP_Z_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_conf);

    // Install central interrupt service dispatcher in IRAM
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3);

    // Register handlers for each axis
    gpio_isr_handler_add(ENDSTOP_X_GPIO, multi_axis_endstop_isr, (void *)&AXIS_X_CFG);
    gpio_isr_handler_add(ENDSTOP_Y_GPIO, multi_axis_endstop_isr, (void *)&AXIS_Y_CFG);
    gpio_isr_handler_add(ENDSTOP_Z_GPIO, multi_axis_endstop_isr, (void *)&AXIS_Z_CFG);
}

