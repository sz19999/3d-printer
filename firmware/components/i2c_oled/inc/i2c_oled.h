#ifndef I2C_OLED_H
#define I2C_OLED_H

#include <stdint.h>

void oled_i2c_write(const uint8_t *data, size_t data_len);
void oled_clear_screen(void);
void oled_init(void);

#endif