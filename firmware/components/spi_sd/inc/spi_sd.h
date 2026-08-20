#ifndef SPI_SD_H
#define SPI_SD_H

#include <stdio.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#define MOUNT_POINT "/sdcard"

#define PIN_MISO GPIO_NUM_13  // FSPIQ
#define PIN_MOSI GPIO_NUM_11  // FSPID
#define PIN_CLK  GPIO_NUM_12  // FSPICLK
#define PIN_CS   GPIO_NUM_10  // FSPICS0

void sd_init(sdmmc_card_t** card, sdmmc_host_t* host);
void sd_read_line(FILE* f);
FILE* open_first_by_extension(const char *search_dir, const char *target_ext, const char *mode);
bool has_extension(const char *filename, const char *ext);

#endif