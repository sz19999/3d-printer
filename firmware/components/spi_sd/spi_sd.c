#include "spi_sd.h"
#include <stdbool.h>
#include <string.h>
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <errno.h>
#include <dirent.h>

static const char *TAG = "SD_CARD_TEST";

void sd_init(sdmmc_card_t** card, sdmmc_host_t* host) {
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing SD card via SPI (FSPI)...");

    // 1. Configure FatFs Mount Options
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 0
    };

    host->slot = SPI2_HOST; // Select FSPI Peripheral
    host->max_freq_khz = SDMMC_FREQ_DEFAULT;  // 20MHz speed

    // 2. Configure SPI Bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(host->slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus (%s)", esp_err_to_name(ret));
        return;
    }

    // 3. Configure Slot / CS Pin
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_CS;
    slot_config.host_id = host->slot;

    // 4. Mount File System
    ESP_LOGI(TAG, "Mounting FAT filesystem...");
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, host, &slot_config, &mount_config, card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. Format SD card to FAT32.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize card (%s). Check wiring/power.", esp_err_to_name(ret));
        }
        spi_bus_free(host->slot);
        return;
    }

    // 5. Card Information Log
    ESP_LOGI(TAG, "SD Card mounted successfully!");
    sdmmc_card_print_info(stdout, *card);
}

void sd_read_line(FILE* f) {
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file. errno: %d (%s)", errno, strerror(errno));
    } else {
        char buffer[128];
        if (fgets(buffer, sizeof(buffer), f) != NULL) {
            ESP_LOGI(TAG, "Read from SD card: %s", buffer);
        } else {
            ESP_LOGE(TAG, "Failed to read data from file");
        }
    }
}