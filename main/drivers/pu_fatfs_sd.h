
#ifndef _DISKIO_DEFINED_SD
#define _DISKIO_DEFINED_SD

#define _USE_WRITE 1 /* 1: Enable disk_write function */
#define _USE_IOCTL 1 /* 1: Enable disk_ioctl fucntion */

#include "stm32f4xx.h"
#include "diskio.h"
#include "ff.h"

#define PU_FATFS_SPI SPI3
/* Using */
#define PU_FATFS_SPI_MOSI_PIN 5
#define PU_FATFS_SPI_MOSI_PORT GPIOC
#define PU_FATFS_SPI_MISO_PIN 4
#define PU_FATFS_SPI_MISO_PORT GPIOC
#define PU_FATFS_SPI_SCK_PIN 3
#define PU_FATFS_SPI_SCK_PORT GPIOB

#define PU_FATFS_CS_PORT GPIOB
#define PU_FATFS_CS_PIN 0

#define MMC_WP 0 /* Write protected (yes:true, no:false, default:false) */

#define PU_FATFS_USE_DETECT_PIN 0
#define PU_FATFS_USE_WRITEPROTECT_PIN 0

#define PU_FATFS_CS_LOW PU_FATFS_CS_PORT->BSRRH = PU_FATFS_CS_PIN
#define PU_FATFS_CS_HIGH PU_FATFS_CS_PORT->BSRRL = PU_FATFS_CS_PIN

/* Command code for disk_ioctrl fucntion */

/* Generic command (Used by FatFs) */
#define CTRL_SYNC			0	/* Complete pending write process (needed at FF_FS_READONLY == 0) */
#define GET_SECTOR_COUNT	1	/* Get media size (needed at FF_USE_MKFS == 1) */
#define GET_SECTOR_SIZE		2	/* Get sector size (needed at FF_MAX_SS != FF_MIN_SS) */
#define GET_BLOCK_SIZE		3	/* Get erase block size (needed at FF_USE_MKFS == 1) */
#define CTRL_TRIM			4	/* Inform device that the data on the block of sectors is no longer used (needed at FF_USE_TRIM == 1) */

/* Generic command (Not used by FatFs) */
#define CTRL_FORMAT			5	/* Create physical format on the media */
#define CTRL_POWER_IDLE		6	/* Put the device idle state */
#define CTRL_POWER_OFF		7	/* Put the device off state */
#define CTRL_LOCK			8	/* Lock media removal */
#define CTRL_UNLOCK			9	/* Unlock media removal */
#define CTRL_EJECT			10	/* Eject media */
#define CTRL_GET_SMART		11	/* Read SMART information */

/* MMC/SDC specific command (Not used by FatFs) */
#define MMC_GET_TYPE		50	/* Get card type */
#define MMC_GET_CSD			51	/* Read CSD */
#define MMC_GET_CID			52	/* Read CID */
#define MMC_GET_OCR			53	/* Read OCR */
#define MMC_GET_SDSTAT		54	/* Read SD status */
#define ISDIO_READ			55	/* Read data form SD iSDIO register */
#define ISDIO_WRITE			56	/* Write data to SD iSDIO register */
#define ISDIO_MRITE			57	/* Masked write data to SD iSDIO register */

/* ATA/CF specific command (Not used by FatFs) */
#define ATA_GET_REV			60	/* Get F/W revision */
#define ATA_GET_MODEL		61	/* Get model name */
#define ATA_GET_SN			62	/* Get serial number */


/* MMC card type flags (MMC_GET_TYPE) */
#define CT_MMC		0x01		/* MMC ver 3 */
#define CT_SD1		0x02		/* SD ver 1 */
#define CT_SD2		0x04		/* SD ver 2 */
#define CT_SDC		(CT_SD1|CT_SD2)	/* SD */
#define CT_BLOCK	0x08		/* Block addressing */



#endif