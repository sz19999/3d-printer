#include "tmc2209.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "esp_log.h"


#define TMC2209_UART_PORT      UART_NUM_1
#define TMC2209_BAUD_RATE      115200
#define TMC2209_TX_PIN         GPIO_NUM_17
#define TMC2209_RX_PIN         GPIO_NUM_18
#define TMC2209_READ_TIMEOUT_MS 100

static const char *TAG = "TMC2209";

/*
    Initializes the ESP32 UART peripheral for TMC2209 single-wire bus.
    Wire Setup: ESP32 TX pin -> 1k Ohm resistor -> Joint Bus (TX+RX) -> Driver PDN_UART.
*/
void tmc2209_init_uart(void) {
    uart_config_t uart_config = {
        .baud_rate = TMC2209_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(TMC2209_UART_PORT, 256, 256, 0, NULL, 0);
    uart_param_config(TMC2209_UART_PORT, &uart_config);
    uart_set_pin(TMC2209_UART_PORT, TMC2209_TX_PIN, TMC2209_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

/*
    setups tmc2209 motor driver
*/
void setup_tmc2209(uint8_t driver_addr) {
    // 1. Set current: Hold = ~0.3A, Run = ~0.8A, Delay = 6
    uint32_t current_val = (6 << 16) | (20 << 8) | (8 << 0);
    tmc2209_write_register(driver_addr, 0x10, current_val);

    // 2. Set microstepping: 1/16 step (MRES=4) with 256 interpolation
    uint32_t chopconf_val = 0x10000053 | (4 << 24);
    tmc2209_write_register(driver_addr, 0x6C, chopconf_val);
}

/*
    Calculates the 8-bit CRC required by TMC2209 hardware (Polynomial 0x07, LSB first).
*/
uint8_t tmc2209_calc_crc(const uint8_t *data, uint8_t length) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; i++) {
        uint8_t current_byte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (current_byte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = (crc << 1);
            }
            current_byte >>= 1;
        }
    }
    return crc;
}

/*
    Writes a 32-bit register value to a TMC2209 driver over UART.
*/
bool tmc2209_write_register(uint8_t slave_addr, uint8_t reg_addr, uint32_t value) {
    uint8_t frame[8];

    frame[0] = 0x05;                           // Sync byte
    frame[1] = slave_addr & 0x03;              // Driver address (0 to 3)
    frame[2] = reg_addr | 0x80;                // Bit 7 high = Write operation
    frame[3] = (uint8_t)(value >> 24);         // MSB
    frame[4] = (uint8_t)(value >> 16);
    frame[5] = (uint8_t)(value >> 8);
    frame[6] = (uint8_t)(value & 0xFF);        // LSB
    frame[7] = tmc2209_calc_crc(frame, 7);     // Calculate mandatory CRC

    // Clear input ring buffer before sending
    uart_flush_input(TMC2209_UART_PORT);

    // Send 8-byte datagram
    int bytes_sent = uart_write_bytes(TMC2209_UART_PORT, (const char *)frame, 8);
    
    // Read and discard 8-byte echo caused by tying TX and RX together
    uint8_t echo_buffer[8];
    uart_read_bytes(TMC2209_UART_PORT, echo_buffer, 8, pdMS_TO_TICKS(10));

    return (bytes_sent == 8);
}

/*
    Reads a 32-bit register value from a TMC2209 driver over UART.
 */
bool tmc2209_read_register(uint8_t slave_addr, uint8_t reg_addr, uint32_t *out_value) {
    uint8_t request[4];

    request[0] = 0x05;                         // Sync byte
    request[1] = slave_addr & 0x03;            // Driver address
    request[2] = reg_addr & 0x7F;              // Bit 7 low = Read operation
    request[3] = tmc2209_calc_crc(request, 3); // Request CRC

    uart_flush_input(TMC2209_UART_PORT);

    // 1. Send 4-byte read request
    if (uart_write_bytes(TMC2209_UART_PORT, (const char *)request, 4) != 4) {
        return false;
    }

    // 2. Discard 4-byte transmitted echo
    uint8_t echo[4];
    uart_read_bytes(TMC2209_UART_PORT, echo, 4, pdMS_TO_TICKS(10));

    // 3. Read 8-byte reply from TMC2209
    uint8_t response[8];
    int rx_bytes = uart_read_bytes(TMC2209_UART_PORT, response, 8, pdMS_TO_TICKS(TMC2209_READ_TIMEOUT_MS));

    if (rx_bytes != 8) {
        ESP_LOGE(TAG, "Read timeout or short response (%d bytes)", rx_bytes);
        return false;
    }

    // 4. Validate response CRC
    if (tmc2209_calc_crc(response, 7) != response[7]) {
        ESP_LOGE(TAG, "CRC validation failed on read response!");
        return false;
    }

    // 5. Assemble 32-bit word (Big-Endian from wire)
    *out_value = ((uint32_t)response[3] << 24) |
                 ((uint32_t)response[4] << 16) |
                 ((uint32_t)response[5] << 8)  |
                  (uint32_t)response[6];

    return true;
}