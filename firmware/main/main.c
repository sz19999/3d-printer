#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "UART_TEST";

// define LED pin
#define LED_PIN (GPIO_NUM_4)

// define UART parameters
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_18)
#define UART_PORT_NUM (UART_NUM_1)
#define UART_BAUD_RATE (115200)

// buffer size for incoming data
#define RX_BUF_SIZE (1024)

void init_uart(void) 
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 1. Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));

    // 2. Set UART pins (TX, RX, RTS, CTS)
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // 3. Install UART driver using an internal Rx buffer (No Tx buffer, no event queue)
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, RX_BUF_SIZE, 0, 0, NULL, 0));
}

#define MAX_RECV_LEN 64

void app_main(void)
{
    init_uart();
    ESP_LOGI(TAG, "UART1 Initialized. Starting byte-by-byte loopback test...");

    // Send a newline-terminated string to make parsing easier
    const char *tx_data = "Hello ESP-IDF\n"; 
    char rx_buffer[MAX_RECV_LEN];

    while (1) {
        // 1. Transmit the data
        ESP_LOGI(TAG, "[TX] Sending: %s", tx_data);
        uart_write_bytes(UART_PORT_NUM, tx_data, strlen(tx_data));

        int bytes_read = 0;
        char incoming_byte;
        
        // 2. Read byte-by-byte
        // We loop until we hit a newline, a timeout, or fill our stack buffer
        while (bytes_read < (MAX_RECV_LEN - 1)) {
            // Read exactly 1 byte. We give it a short 10ms timeout.
            int res = uart_read_bytes(UART_PORT_NUM, &incoming_byte, 1, pdMS_TO_TICKS(10));
            
            if (res > 0) {
                rx_buffer[bytes_read++] = incoming_byte;
                if (incoming_byte == '\n') {
                    break; // Stop reading; we hit the end of the message
                }
            } else {
                // res == 0 means timeout (no more bytes currently available)
                break; 
            }
        }

        // 3. Process the results
        if (bytes_read > 0) {
            rx_buffer[bytes_read] = '\0'; // Null-terminate the string safely
            ESP_LOGI(TAG, "[RX] Received %d bytes: %s", bytes_read, rx_buffer);
            
            if (strcmp(tx_data, rx_buffer) == 0) {
                ESP_LOGW(TAG, "Result: SUCCESS!");
            } else {
                ESP_LOGE(TAG, "Result: ERROR! Data mismatch.");
            }
        } else {
            ESP_LOGE(TAG, "Result: ERROR! No bytes received. Check wire!");
        }

        ESP_LOGI(TAG, "--------------------------------------------------");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}