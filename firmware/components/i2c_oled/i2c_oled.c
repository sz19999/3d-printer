#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "i2c_oled.h"

// hardware pin definitions
#define I2C_SDA_GPIO        GPIO_NUM_8
#define I2C_SCL_GPIO        GPIO_NUM_9

// bus configuration parameters
#define OLED_I2C_ADDR       0x3C        // Default I2C address for 4-pin OLED modules
#define OLED_CLOCK_SPEED_HZ 400000      // 400 kHz Fast-mode

static i2c_master_dev_handle_t oled_dev_handle = NULL;  // device handle

void oled_i2c_write(const uint8_t *data, size_t data_len) {
    i2c_master_transmit(oled_dev_handle, data, data_len, -1);
}

void oled_clear_screen(void) {
    // Set Column Address Range (0 to 127)
    const uint8_t set_col[] = { 0x00, 0x21, 0x00, 0x7F };
    oled_i2c_write(set_col, sizeof(set_col));

    // Set Page Address Range (0 to 7)
    const uint8_t set_page[] = { 0x00, 0x22, 0x00, 0x07 };
    oled_i2c_write(set_page, sizeof(set_page));

    // Stream 1,024 zero bytes (Control Byte 0x40 -> Data Mode)
    uint8_t zero_buffer[1025];
    zero_buffer[0] = 0x40;
    memset(&zero_buffer[1], 0x00, 1024);

    oled_i2c_write(zero_buffer, sizeof(zero_buffer));
}

void oled_init(void) {
    // 1. Configure I2C Master Bus on ESP32-S3
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    i2c_new_master_bus(&bus_cfg, &bus_handle);

    // 2. Register SSD1309 Device on I2C Bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OLED_I2C_ADDR,
        .scl_speed_hz = OLED_CLOCK_SPEED_HZ,
    };

    i2c_master_bus_add_device(bus_handle, &dev_cfg, &oled_dev_handle);

    // Allow hardware Power-On Reset (POR) circuit to settle
    vTaskDelay(pdMS_TO_TICKS(20));

    // 3. Command Array: Control Byte (0x00) + Opcode/Parameter Pairs
    static const uint8_t init_cmds[] = {
        0x00,        // Control Byte: Stream Command Mode
        0xAE,        // Display OFF (Sleep Mode)
        0xD5, 0x80,  // Set Oscillator Frequency & Clock Divide Ratio
        0xA8, 0x3F,  // Set Multiplex Ratio (1/64 duty)
        0xD3, 0x00,  // Set Display Offset to 0
        0x40,        // Set Display Start Line to 0
        0x20, 0x00,  // Set Memory Addressing Mode to Horizontal
        0xA1,        // Set Segment Re-map (Horizontal Flip)
        0xC8,        // Set COM Output Scan Direction (Vertical Flip)
        0xDA, 0x12,  // Set COM Pins Hardware Configuration
        0x81, 0x7F,  // Set Contrast Level
        0xD9, 0xF1,  // Set Pre-Charge Period
        0xDB, 0x34,  // Set VCOMH Deselect Level
        0xA4,        // Output follows GDDRAM content
        0xA6,        // Normal Display Mode (Non-inverted)
        0xAF         // Display ON
    };

    oled_i2c_write(init_cmds, sizeof(init_cmds));

    // 4. Wipe display memory noise on boot
    oled_clear_screen();
}