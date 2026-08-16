#ifndef I2C_OLED_H
#define I2C_OLED_H

#include <stdint.h>
#include <stddef.h>

#define I2C_SDA_GPIO        GPIO_NUM_8
#define I2C_SCL_GPIO        GPIO_NUM_9

#define OLED_I2C_ADDR       0x3C        // Default I2C address for 4-pin OLED modules
#define OLED_CLOCK_SPEED_HZ 400000      // 400 kHz Fast-mode

#define OLED_WIDTH  128
#define OLED_HEIGHT 64

void oled_i2c_write(const uint8_t *data, size_t data_len);
void oled_clear_screen(void);
void oled_init(void);
void oled_flush(void);
void oled_draw_pixel(int32_t x, int32_t y, int32_t color);
void oled_draw_char(int32_t x, int32_t y, uint8_t c);
void oled_draw_string(int32_t x, int32_t y, char* str);
void oled_print_line(int32_t line, char* str);
void oled_highlight_line(int32_t line);

#endif