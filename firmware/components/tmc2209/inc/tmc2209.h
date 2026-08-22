#ifndef TMC2209_H
#define TMC2209_H

#include <stdint.h>
#include <stdbool.h>

#define TMC2209_UART_PORT      UART_NUM_1
#define TMC2209_BAUD_RATE      115200
#define TMC2209_TX_PIN         GPIO_NUM_17
#define TMC2209_RX_PIN         GPIO_NUM_18
#define TMC2209_READ_TIMEOUT_MS 100
#define TMC2209_DRIVER_ADDR    0x00          // Address 0 (MS1 & MS2 connected to GND)

// Registers:
#define TMC2209_REG_GCONF      0x00
#define TMC2209_REG_IHOLD_IRUN 0x10
#define TMC2209_REG_VACTUAL    0x22
#define TMC2209_REG_CHOPCONF   0x6C
#define TMC2209_REG_IOIN       0x06


void tmc2209_init_uart(void);
void setup_tmc2209(uint8_t driver_addr);
bool tmc2209_write_register(uint8_t slave_addr, uint8_t reg_addr, uint32_t value);
bool tmc2209_read_register(uint8_t slave_addr, uint8_t reg_addr, uint32_t *out_value);
uint8_t tmc2209_calc_crc(const uint8_t *data, uint8_t length);

#endif