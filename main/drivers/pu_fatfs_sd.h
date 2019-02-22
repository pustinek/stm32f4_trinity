
#ifndef _DISKIO_DEFINED_SD
#define _DISKIO_DEFINED_SD

#define _USE_WRITE 1 /* 1: Enable disk_write function */
#define _USE_IOCTL 1 /* 1: Enable disk_ioctl fucntion */

#include "stm32f4xx.h"
#include "diskio.h"
#include "ff.h"

#define PU_FATFS_SPI SPI3
/* Using */
#define PU_FATFS_SPI_PORT GPIOB
#define PU_FATFS_SPI_MOSI_PIN 5
#define PU_FATFS_SPI_MISO_PIN 4
#define PU_FATFS_SPI_CLK_PIN 3

#define PU_FATFS_CS_PIN 6
#define PU_FATFS_CS_PORT GPIOB

#define PU_FATFS_USE_DETECT_PIN 0
#define PU_FATFS_USE_WRITEPROTECT_PIN 0

#define PU_FATFS_CS_LOW FATFS_CS_PORT->BSRRH = FATFS_CS_PIN
#define PU_FATFS_CS_HIGH FATFS_CS_PORT->BSRRL = FATFS_CS_PIN

#endif