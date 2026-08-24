#include <stdint.h>
#include "sys_state_machine.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

extern QueueHandle_t gpio_evt_queue;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    ButtonEdge_t edge;
    edge.is_pressed = (gpio_get_level(BTN_PIN) == 0); // Active LOW
    edge.timestamp_us = esp_timer_get_time();

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(gpio_evt_queue, &edge, &xHigherPriorityTaskWoken);
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void init_button_interrupt(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BTN_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE // Trigger on both press and release
    };
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(ButtonEdge_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_PIN, gpio_isr_handler, NULL);
}

ButtonEvent_t process_button_edges(void) {
    static enum { STATE_IDLE, STATE_PRESSED, STATE_WAIT_SECOND } state = STATE_IDLE;
    static int64_t press_start_us = 0;
    static int64_t release_time_us = 0;

    ButtonEdge_t edge;
    int64_t now_us = esp_timer_get_time();

    // Pull edge from queue without blocking (0 ticks timeout)
    if (xQueueReceive(gpio_evt_queue, &edge, 0) == pdTRUE) {
        
        // Basic Software Debounce (Ignore edges within 30ms)
        static int64_t last_edge_time = 0;
        if ((edge.timestamp_us - last_edge_time) < 30000) return EVENT_NONE;
        last_edge_time = edge.timestamp_us;

        if (edge.is_pressed) {
            if (state == STATE_IDLE) {
                press_start_us = edge.timestamp_us;
                state = STATE_PRESSED;
            } else if (state == STATE_WAIT_SECOND) {
                state = STATE_IDLE;
                return EVENT_DOUBLE_CLICK; // 2nd press within window -> UP
            }
        } else { // Released
            if (state == STATE_PRESSED) {
                int64_t hold_ms = (edge.timestamp_us - press_start_us) / 1000;
                if (hold_ms >= 800) {
                    state = STATE_IDLE;
                    return EVENT_LONG_PRESS; // Held >= 800ms -> CONFIRM
                } else {
                    release_time_us = edge.timestamp_us;
                    state = STATE_WAIT_SECOND; // Wait for possible second press
                }
            }
        }
    }

    // Handle double-click window timeout passively
    if (state == STATE_WAIT_SECOND) {
        if ((now_us - release_time_us) / 1000 >= 350) { // 250ms window expired
            state = STATE_IDLE;
            return EVENT_SINGLE_CLICK; // Single press confirmed -> DOWN
        }
    }

    return EVENT_NONE;
}