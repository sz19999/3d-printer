#include "tmc2209_test.h"
#include "tmc2209.h"
#include <stdint.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static const char *TAG = "TMC2209";

void tmc2209_set_velocity(int32_t velocity);

void spin_motor(void) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "Initializing Single-Wire UART for TMC2209...");

    gpio_reset_pin(GPIO_NUM_16);
    gpio_set_direction(GPIO_NUM_16, GPIO_MODE_INPUT);

    tmc2209_init_uart();

    setup_tmc2209(TMC2209_DRIVER_ADDR);

    vTaskDelay(pdMS_TO_TICKS(500));

    uint32_t reg_val = 0;
    // 1. Read IOIN register (Read-Only live pin status)
    if (tmc2209_read_register(TMC2209_DRIVER_ADDR, TMC2209_REG_IOIN, &reg_val)) {
        ESP_LOGI(TAG, "[PASS] Read IOIN (0x06): 0x%08LX", reg_val);
        uint8_t version = (reg_val >> 24) & 0xFF; // Bits 31..24 store IC version
        ESP_LOGI(TAG, "       -> TMC2209 IC Version: 0x%02X (Expected: 0x21)", version);
    } else {
        ESP_LOGE(TAG, "[FAIL] Failed to read IOIN register!");
    }
    
    while (1) {
        ESP_LOGI(TAG, "Spinning FORWARD (VACTUAL = 5,000)...");
        tmc2209_set_velocity(10000);  // ~1-2 RPM depending on microsteps
        vTaskDelay(pdMS_TO_TICKS(3000));

        ESP_LOGI(TAG, "Stopping motor...");
        tmc2209_set_velocity(0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "Spinning REVERSE (VACTUAL = -5,000)...");
        tmc2209_set_velocity(-10000); // Negative value for reverse
        vTaskDelay(pdMS_TO_TICKS(3000));

        ESP_LOGI(TAG, "Stopping motor...");
        tmc2209_set_velocity(0);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void tmc2209_set_velocity(int32_t velocity) {
    tmc2209_write_register(TMC2209_DRIVER_ADDR, TMC2209_REG_VACTUAL, (uint32_t)velocity);
}