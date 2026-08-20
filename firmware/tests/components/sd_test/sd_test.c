
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <errno.h>
#include <dirent.h>
#include "spi_sd.h"

static const char *TAG = "SD_CARD_TEST";

void read_line(void) {
    sdmmc_card_t *card = NULL;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    const char *filepath = MOUNT_POINT "/cube.gcode";

    sd_init(&card, &host);
    
    // read a line
    FILE* f = fopen(filepath, "rb");
    ESP_LOGI(TAG, "Opening file %s for reading...", filepath);
    sd_read_line(f);
    fclose(f);
    
    // Clean Unmount
    esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
    spi_bus_free(host.slot);
    ESP_LOGI(TAG, "Card unmounted cleanly. Test Complete.");
}