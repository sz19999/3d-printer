#include "step_gen_test.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "step_generator.h"
#include "driver/rmt_tx.h"
#include "hal/gpio_ll.h"
#include "soc/rmt_struct.h"
#include "esp_log.h"


const char* TAG = "STEP_GEN_TEST";

// Per-axis configuration instances
static const axis_endstop_config_t AXIS_X_CFG = { .axis_id = 0, .step_pin = STEP_PIN_X, .rmt_channel = RMT_CHANNEL_X };
static const axis_endstop_config_t AXIS_Y_CFG = { .axis_id = 1, .step_pin = STEP_PIN_Y, .rmt_channel = RMT_CHANNEL_Y };
static const axis_endstop_config_t AXIS_Z_CFG = { .axis_id = 2, .step_pin = STEP_PIN_Z, .rmt_channel = RMT_CHANNEL_Z };

// Global handle to the step generator task
static TaskHandle_t s_step_task_handle = NULL;

static void IRAM_ATTR multi_axis_endstop_isr(void *arg) {
    // Cast void argument back to specific axis context
    const axis_endstop_config_t *axis = (const axis_endstop_config_t *)arg;

    // 1. HARDWARE OVERRIDE: Immediately force step pin LOW via Low-Layer HAL
    gpio_ll_set_level(&GPIO, axis->step_pin, 0);
    gpio_ll_output_enable(&GPIO, axis->step_pin);

    // 2. PERIPHERAL SHUTDOWN: Reset RMT hardware registers directly
    RMT.conf_ch[axis->rmt_channel].conf1.tx_start   = 0; // Stop transmission
    RMT.conf_ch[axis->rmt_channel].conf1.mem_rd_rst = 1; // Pulse read pointer reset high
    RMT.conf_ch[axis->rmt_channel].conf1.mem_rd_rst = 0; // Release reset

    // 3. SOFTWARE NOTIFICATION: Unblock Step Generator Task via bitmask
    if (s_step_task_handle != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // Set bit corresponding to axis ID (Bit 0 = X, Bit 1 = Y, Bit 2 = Z)
        xTaskNotifyFromISR(
            s_step_task_handle,
            (1 << axis->axis_id),
            eSetBits,
            &xHigherPriorityTaskWoken
        );

        // Yield immediately if Step Task has higher priority than current execution context
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

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

// 1. Configure endstop GPIO pins & install the shared ISR
void init_endstops(void) {

    // Configure inputs with pull-ups (Active-LOW switches)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ENDSTOP_X_GPIO) | 
                        (1ULL << ENDSTOP_Y_GPIO) | 
                        (1ULL << ENDSTOP_Z_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
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

TaskHandle_t start_step_generator_task(void) {
    xTaskCreate(step_generator_task, "step_task", 4096, NULL, 3, &s_step_task_handle);
    return s_step_task_handle;
}

void endstops_test(void) {
    ESP_LOGI(TAG, "Initializing System...");

    // 1. Start the step engine task
    start_step_generator_task();

    // 2. Setup ISR endstop interrupts
    init_endstops();

    ESP_LOGI(TAG, "Step generator running and ISRs active.");
}