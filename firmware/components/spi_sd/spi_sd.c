#include <errno.h>
#include <dirent.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>

#include "spi_sd.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

static const char *TAG = "SD_CARD_TEST";
static const char *TAG_F = "FILE_SEARCH";

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


// check if a filename ends with a given extension
bool has_extension(const char *filename, const char *ext) {
    size_t filename_len = strlen(filename);
    size_t ext_len = strlen(ext);

    if (filename_len < ext_len) {
        return false;
    }

    const char *str1 = filename + (filename_len - ext_len);
    const char *str2 = ext;

    while (*str1 != '\0' && *str2 != '\0') {
        if (tolower((unsigned char)*str1) != tolower((unsigned char)*str2)) {
            return false;
        }
        str1++;
        str2++;
    }

    return true;
}

// Searches directory for the first file matching target_ext and opens it
FILE* open_first_by_extension(const char *search_dir, const char *target_ext, const char *mode) {
    DIR *dir = opendir(search_dir);
    if (!dir) {
        ESP_LOGE(TAG_F, "Failed to open directory: %s", search_dir);
        return NULL;
    }

    struct dirent *entry;
    char full_path[512];
    bool found = false;

    while ((entry = readdir(dir)) != NULL) {
        ESP_LOGI(TAG_F, "Raw directory entry: '%s' (len: %d)", entry->d_name, strlen(entry->d_name));

        // skip directories or hidden files
        if (entry->d_type == DT_DIR) continue;

        if (has_extension(entry->d_name, target_ext)) {
            snprintf(full_path, sizeof(full_path), "%s/%s", search_dir, entry->d_name);
            ESP_LOGI(TAG_F, "Found matching file: %s", entry->d_name);
            found = true;
            break; // stop at the first matching file
        }
    }

    closedir(dir);

    if (found) {
        FILE *file = fopen(full_path, mode);
        if (!file) {
            ESP_LOGE(TAG_F, "Failed to open file: %s", full_path);
        }
        return file;
    }

    ESP_LOGW(TAG_F, "No file with extension '%s' found in %s", target_ext, search_dir);
    return NULL;
}