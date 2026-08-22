#ifndef TMC2209_H
#define TMC2209_H

void tmc2209_init_uart(void);
void setup_tmc2209(uint8_t driver_addr);
bool tmc2209_write_register(uint8_t slave_addr, uint8_t reg_addr, uint32_t value);
bool tmc2209_read_register(uint8_t slave_addr, uint8_t reg_addr, uint32_t *out_value);
uint8_t tmc2209_calc_crc(const uint8_t *data, uint8_t length);

#endif