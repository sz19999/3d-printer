#include "step_gen_test.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "step_generator.h"
#include "driver/rmt_tx.h"
#include "hal/gpio_ll.h"
#include "soc/rmt_struct.h"
#include "esp_log.h"


void step_generator_task(void *pvParameters) {
    uint32_t abort_bitmask = 0;

    rmt_channel_handle_t rmt_channels[3];
    gpio_num_t step_pins[3] = { STEP_PIN_X, STEP_PIN_Y, STEP_PIN_Z };

    // 1. Initialize RMT channels
    for (int i = 0; i < 3; i++) {
        rmt_tx_channel_config_t tx_chan_config = {
            .gpio_num = step_pins[i],
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = 1000000, // 1 MHz resolution (1 tick = 1 microsecond)
            .mem_block_symbols = 64,
            .trans_queue_depth = 4,
        };
        ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &rmt_channels[i]));
        ESP_ERROR_CHECK(rmt_enable(rmt_channels[i]));
    }

    // 2. Create modern copy encoder
    rmt_encoder_handle_t copy_encoder = NULL;
    rmt_copy_encoder_config_t encoder_config = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&encoder_config, &copy_encoder));

    // 3. Define 20 Hz step symbol for LED testing (50% duty cycle: 25ms HIGH, 25ms LOW)
    rmt_symbol_word_t step_symbol = (rmt_symbol_word_t) {
        .duration0 = 25000, .level0 = 1, // 25,000 us HIGH
        .duration1 = 25000, .level1 = 0  // 25,000 us LOW
    };

    // 4. Configure HARDWARE LOOPING
    rmt_transmit_config_t tx_config = {
        .loop_count = -1, 
    };

    ESP_LOGI(TAG, "Starting hardware-looped 20Hz step pulse generation...");

    // Start infinite hardware transmission ONCE on all active channels
    for (int i = 0; i < 3; i++) {
        rmt_transmit(rmt_channels[i], copy_encoder, &step_symbol, sizeof(step_symbol), &tx_config);
    }

    // 5. Task blocks and waits strictly for endstop abort notifications
    while (1) {
        if (xTaskNotifyWait(0, ULONG_MAX, &abort_bitmask, 0) == pdTRUE) {
            
            if (abort_bitmask & (1 << AXIS_X_CFG.axis_id)) {
                ESP_LOGE(TAG, ">>> ABORT: AXIS X ENDSTOP TRIPPED! <<<");
                // Stop the driver channel in software as well
                rmt_disable(rmt_channels[0]);

            }
            if (abort_bitmask & (1 << AXIS_Y_CFG.axis_id)) {
                ESP_LOGE(TAG, ">>> ABORT: AXIS Y ENDSTOP TRIPPED! <<<");
                rmt_disable(rmt_channels[1]);
            }
            if (abort_bitmask & (1 << AXIS_Z_CFG.axis_id)) {
                ESP_LOGE(TAG, ">>> ABORT: AXIS Z ENDSTOP TRIPPED! <<<");
                rmt_disable(rmt_channels[2]);
            }
        }

        // generate pulses 

        vTaskDelay(1);
    }
}

void endstops_test(void) {
    init_endstops();

}