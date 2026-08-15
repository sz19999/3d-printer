#include <stdint.h>
#include "oled_test.h"

void oled_test_fill(int enable) {
    // 0xA5 = Entire Display ON (Forces all pixels ON)
    // 0xA4 = Resume to RAM Content
    const uint8_t cmd[] = { 0x00, enable ? 0xA5 : 0xA4 };
    oled_i2c_write(cmd, sizeof(cmd));
}