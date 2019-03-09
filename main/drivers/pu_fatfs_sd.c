/*-----------------------------------------------------------------------*/
/* SPI controls (Platform dependent)                                     */
/*-----------------------------------------------------------------------*/

#include "pu_fatfs_sd.h"
#include "pu_spi_driver.h"
#include "pu_gpio_driver.h"
#include "spi_main.h"
#include "diskio.h"
/* MMC/SD command */
#define CMD0 (0)           /* GO_IDLE_STATE */
#define CMD1 (1)           /* SEND_OP_COND (MMC) */
#define ACMD41 (0x80 + 41) /* SEND_OP_COND (SDC) */
#define CMD8 (8)           /* SEND_IF_COND */
#define CMD9 (9)           /* SEND_CSD */
#define CMD10 (10)         /* SEND_CID */
#define CMD12 (12)         /* STOP_TRANSMISSION */
#define ACMD13 (0x80 + 13) /* SD_STATUS (SDC) */
#define CMD16 (16)         /* SET_BLOCKLEN */
#define CMD17 (17)         /* READ_SINGLE_BLOCK */
#define CMD18 (18)         /* READ_MULTIPLE_BLOCK */
#define CMD23 (23)         /* SET_BLOCK_COUNT (MMC) */
#define ACMD23 (0x80 + 23) /* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24 (24)         /* WRITE_BLOCK */
#define CMD25 (25)         /* WRITE_MULTIPLE_BLOCK */
#define CMD32 (32)         /* ERASE_ER_BLK_START */
#define CMD33 (33)         /* ERASE_ER_BLK_END */
#define CMD38 (38)         /* ERASE */
#define CMD55 (55)         /* APP_CMD */
#define CMD58 (58)         /* READ_OCR */

pu_spi_handle_t pu_fatfs_spi_handle;

static volatile DSTATUS Stat = STA_NOINIT; /* Physical drive status */
static volatile UINT Timer1, Timer2;       /* 1kHz decrement timer stopped at zero (disk_timerproc()) */
static BYTE CardType;                      /* Card type flags */

/**
	* @brief  Initialize gpio for use with spi
	* @param  None
	* @retval None
*/
static void pu_fatfs_init_gpio(void)
{
    static gpio_pin_conf_t spi_conf;
    /* Enable clock for port A*/
    _HAL_RCC_GPIOB_CLK_ENABLE();
    /* Configure GPIOB_PIN_5 for spi SCK functionality */
    spi_conf.pin = SPI_CLK_PIN;
    spi_conf.mode = GPIO_PIN_ALT_FUN_MODE;
    spi_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
    spi_conf.pull = GPIO_PIN_PULL_DOWN;
    spi_conf.speed = GPIO_PIN_SPEED_MEDIUM;
    pu_gpio_set_alt_function(PU_FATFS_SPI_SCK_PORT, PU_FATFS_SPI_SCK_PIN, GPIO_PIN_AF5_SPI2);
    pu_gpio_init(PU_FATFS_SPI_SCK_PORT, &spi_conf);

    /* configure GPIOB_PIN_5 for SPI MOSI functionality */
    spi_conf.pin = PU_FATFS_SPI_MOSI_PIN;
    spi_conf.pull = GPIO_PIN_PULL_UP;
    pu_gpio_set_alt_function(GPIOB, PU_FATFS_SPI_MOSI_PIN, GPIO_PIN_AF5_SPI2);
    pu_gpio_init(PU_FATFS_SPI_MOSI_PORT, &spi_conf);

    /* configure GPIOB_PIN_4 for SPI MISO functionality */
    spi_conf.pin = PU_FATFS_SPI_MISO_PIN;
    spi_conf.pull = GPIO_PIN_PULL_UP;
    pu_gpio_set_alt_function(GPIOB, PU_FATFS_SPI_MISO_PIN, GPIO_PIN_AF5_SPI2);
    pu_gpio_init(PU_FATFS_SPI_MISO_PORT, &spi_conf);
}
/**
	* @brief  Initialize gpio for CS pin
	* @param  None
	* @retval None
*/
static void pu_fatfs_init_cs_gpio(void)
{
    static gpio_pin_conf_t cs_conf;
    cs_conf.pin = PU_FATFS_CS_PIN;
    cs_conf.mode = GPIO_PIN_OUTPUT_MODE;
    cs_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
    cs_conf.speed = GPIO_PIN_SPEED_MEDIUM;
    cs_conf.pull = GPIO_PIN_NO_PULL_PUSH;

    pu_gpio_init(PU_FATFS_CS_PORT, &cs_conf);
}

void cs_set(GPIO_TypeDef *GPIOx, uint16_t pin)
{
    pu_gpio_write_to_pin(GPIOx, pin, 1);
}

/**
	* @brief  Initialize spi
	* @param  None
	* @retval None
*/
void pu_fatfs_init_spi(void)
{

    /*TODO:
        - delay
        - init spi
        - set cs high
        - delay, to wait for it to get stable
    */

    /*
		- Using SPI1 device, running @500kHz, 
		- data format is 8bit MSB first
		- SPI mode = 1, because CPO=0 and CPHASE=1.
	*/

    _PU_RCC_SPI2_CLK_ENABLE();
    pu_fatfs_init_gpio();

    /* fill up the handle structure */
    pu_fatfs_spi_handle.Instance = PU_FATFS_SPI;
    pu_fatfs_spi_handle.Init.BaudRatePrescaler = PU_SPI_REG_CR1_BR_PCLK_DIV_32; //16MHz / 32 = 500kHz
    pu_fatfs_spi_handle.Init.Direction = PU_SPI_ENABLE_2_LINE_UNI_DIR;
    pu_fatfs_spi_handle.Init.CLKPhase = PU_SPI_SECOND_CLOCK_TRANS;
    pu_fatfs_spi_handle.Init.CLKPolarity = PU_SPI_CPOL_LOW;
    pu_fatfs_spi_handle.Init.DataSize = PU_SPI_8BIT_DF;
    pu_fatfs_spi_handle.Init.FirstBit = PU_SPI_TX_MSB_FIRST;
    pu_fatfs_spi_handle.Init.NSS = PU_SPI_SSM_ENABLE;
    pu_fatfs_spi_handle.Init.Mode = PU_SPI_MASTER_MODE_SEL;

    pu_fatfs_spi_handle.State = PU_SPI_STATE_READY;

    pu_fatfs_init_cs_gpio();
    pu_spi_init(&pu_fatfs_spi_handle);

    NVIC_EnableIRQ(SPI2_IRQn);

    PU_FATFS_CS_HIGH; /* Set CS pin High */
    for (Timer1 = 10; Timer1;)
        ; /* 10ms */
}

/**
	* @brief  Exchange a byte
	* @param  data : Data to send
	* @retval Recieved byte
*/
static BYTE xchg_spi(BYTE data)
{

    uint8_t ack_buf[2];

    pu_spi_master_tx(&pu_fatfs_spi_handle, &data, 8);
    while (pu_fatfs_spi_handle.State != PU_SPI_STATE_READY)
        ;
    for (int i = 0; i < 10; i++)
        ;
    pu_spi_master_rx(&pu_fatfs_spi_handle, ack_buf, 2);
    return (BYTE)pu_fatfs_spi_handle.Instance->DR;
}
/**
	* @brief  Recieve multiple bytes
	* @param  *buff : Pointer to data buffer
    * @param  btr : Number of bytes to receive
	* @retval None
*/
static void rcvr_spi_multi(BYTE *buff, UINT btr)
{
    WORD d;
    pu_fatfs_spi_handle.Init.DataSize = PU_SPI_16BIT_DF;
    pu_spi_master_tx(&pu_fatfs_spi_handle, (uint8_t)0xFFFF, 16);
    while (pu_fatfs_spi_handle.State != PU_SPI_STATE_READY)
        ; /* Wait to send all the data */
    pu_spi_master_rx(&pu_fatfs_spi_handle, buff, btr);
    while (pu_fatfs_spi_handle.State != PU_SPI_STATE_READY)
        ; /* Wait to receive all the data */
    pu_fatfs_spi_handle.Init.DataSize = PU_SPI_8BIT_DF;
}
/**
	* @brief  Send multiple bytes
	* @param  *buff : Pointer to data buffer
    * @param  btr : Number of bytes to send 
	* @retval None
*/
static void xmit_spi_multi(BYTE *buff, UINT btx)
{
    pu_fatfs_spi_handle.Init.DataSize = PU_SPI_16BIT_DF;
    pu_spi_master_tx(&pu_fatfs_spi_handle, buff, 16);
    while (pu_fatfs_spi_handle.State != PU_SPI_STATE_READY)
        ;
    pu_fatfs_spi_handle.Init.DataSize = PU_SPI_8BIT_DF;
}

/**
	* @brief  Wait for the card to get ready
	* @param  wt : Timeout [ms]
	* @retval Recieved byte
*/
static int wait_ready(UINT wt)
{
    BYTE d;

    Timer2 = wt;
    do
    {
        d = xchg_spi(0xFF);
        /* This loop takes a time. Insert rot_rdq() here for multitask envilonment. */
    } while (d != 0xFF && Timer2); /* Wait for card goes ready or timeout */

    return (d == 0xFF) ? 1 : 0;
}

/**
	* @brief  Deselect and release SPI
	* @param  None
	* @retval None
*/
static void deselect(void)
{
    PU_FATFS_CS_HIGH; /* Set CS# high */
    xchg_spi(0xFF);   /* Dummy clock (force DO hi-z for multiple slave SPI) */
}

/**
	* @brief  Select card and wait for ready
	* @param  None
	* @retval boolean : 1:OK, 0:Timeout
*/
static int select(void)
{
    PU_FATFS_CS_LOW;
    xchg_spi(0xFF); /* Dummy clock (forece DO enabled) */
    if (wait_ready(500))
        return 1;
    deselect();
    return 0; /* Timeout */
}

/**
	* @brief  Receive a data packet from MMC
	* @param  *buff: Data buffer
    * @param  btr: Data block length(byte)
	* @retval boolean : 1:OK, 0:Timeout
*/
static int rcvr_datablock(BYTE *buff, UINT btr)
{
    BYTE token;

    Timer1 = 200;
    do
    {
        token = xchg_spi(0xFF);
    } while ((token != 0xFE) && Timer1);

    /* Function fails if invalid DataStart token or timeout */
    if (token != 0xFE)
        return 0;
    rcvr_spi_multi(buff, btr); /* Store trailing data to the buffer */
    xchg_spi(0xFF);
    xchg_spi(0xFF); /* Discard CRC */

    return 1; /* Function succeeded */
}

/**
	* @brief  Send a data packet to the MMC
	* @param  *buff: Data buffer
    * @param  token: token
	* @retval boolean : 1:OK, 0:Timeout
*/
#if FF_FS_READONLY == 0
static int xmit_datablock(const BYTE *buff, BYTE token)
{
    BYTE resp;

    if (!wait_ready(500))
        return 0; /* Wait for card ready */

    xchg_spi(token); /* Send token */
    if (token != 0xFD)
    {                              /* Send data if token is other than StopTran */
        xmit_spi_multi(buff, 512); /* Data */
        xchg_spi(0xFF);
        xchg_spi(0xFF); /* Dummy CRC */

        resp = xchg_spi(0xFF); /* Receive data resp */
        if ((resp & 0x1F) != 0x05)
            return 0; /* Function fails if the data packet was not accepted */
    }
    return 1;
}
#endif

/**
	* @brief  Send a command packet to the MMC
	* @param  cmd: command index
    * @param  arg: argument
	* @retval Return value: R1 resp (bit7==1:Failed to send)
*/
static BYTE send_cmd(BYTE cmd, DWORD arg)
{
    BYTE n, res;

    if (cmd & 0x80)
    { /* Send a CMD55 prior to ACMD<n> */
        cmd &= 0x7F;
        res = send_cmd(CMD55, 0);
        if (res > 1)
            return res;
    }

    /* Select the card and wait for ready except to stop multiple block read */
    if (cmd != CMD12)
    {
        deselect();
        if (!select())
            return 0xFF;
    }

    /* Send command packet */
    xchg_spi(0x40 | cmd);        /* Start + command index */
    xchg_spi((BYTE)(arg >> 24)); /* Argument[31..24] */
    xchg_spi((BYTE)(arg >> 16)); /* Argument[23..16] */
    xchg_spi((BYTE)(arg >> 8));  /* Argument[15..8] */
    xchg_spi((BYTE)arg);         /* Argument[7..0] */
    n = 0x01;                    /* Dummy CRC + Stop */
    if (cmd == CMD0)
        n = 0x95; /* Valid CRC for CMD0(0) */
    if (cmd == CMD8)
        n = 0x87; /* Valid CRC for CMD8(0x1AA) */
    xchg_spi(n);

    /* Receive command resp */
    if (cmd == CMD12)
        xchg_spi(0xFF); /* Diacard following one byte when CMD12 */
    n = 10;             /* Wait for response (10 bytes max) */
    do
    {
        res = xchg_spi(0xFF);
    } while ((res & 0x80) && --n);

    return res; /* Return received response */
}

/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* Initialize disk drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize(
    BYTE drv /* Physical drive number (0) */
)
{
    BYTE n, cmd, ty, ocr[4];

    if (drv)
        return STA_NOINIT; /* Supports only drive 0 */
    pu_fatfs_init_spi();   /* Initialize SPI */

    if (Stat & STA_NODISK)
        return Stat; /* Is card existing in the soket? */

    // FCLK_SLOW();
    for (n = 10; n; n--)
        xchg_spi(0xFF); /* Send 80 dummy clocks */

    ty = 0;
    if (send_cmd(CMD0, 0) == 1)
    {                  /* Put the card SPI/Idle state */
        Timer1 = 1000; /* Initialization timeout = 1 sec */
        if (send_cmd(CMD8, 0x1AA) == 1)
        { /* SDv2? */
            for (n = 0; n < 4; n++)
                ocr[n] = xchg_spi(0xFF); /* Get 32 bit return value of R7 resp */
            if (ocr[2] == 0x01 && ocr[3] == 0xAA)
            { /* Is the card supports vcc of 2.7-3.6V? */
                while (Timer1 && send_cmd(ACMD41, 1UL << 30))
                    ; /* Wait for end of initialization with ACMD41(HCS) */
                if (Timer1 && send_cmd(CMD58, 0) == 0)
                { /* Check CCS bit in the OCR */
                    for (n = 0; n < 4; n++)
                        ocr[n] = xchg_spi(0xFF);
                    ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2; /* Card id SDv2 */
                }
            }
        }
        else
        { /* Not SDv2 card */
            if (send_cmd(ACMD41, 0) <= 1)
            { /* SDv1 or MMC? */
                ty = CT_SD1;
                cmd = ACMD41; /* SDv1 (ACMD41(0)) */
            }
            else
            {
                ty = CT_MMC;
                cmd = CMD1; /* MMCv3 (CMD1(0)) */
            }
            while (Timer1 && send_cmd(cmd, 0))
                ;                                     /* Wait for end of initialization */
            if (!Timer1 || send_cmd(CMD16, 512) != 0) /* Set block length: 512 */
                ty = 0;
        }
    }
    CardType = ty; /* Card type */
    deselect();

    if (ty)
    {                        /* OK */
                             // FCLK_FAST();         /* Set fast clock */
        Stat &= ~STA_NOINIT; /* Clear STA_NOINIT flag */
    }
    else
    { /* Failed */
        Stat = STA_NOINIT;
    }

    return Stat;
}

/*-----------------------------------------------------------------------*/
/* Get disk status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status(
    BYTE drv /* Physical drive number (0) */
)
{
    if (drv)
        return STA_NOINIT; /* Supports only drive 0 */

    return Stat; /* Return disk status */
}

/*-----------------------------------------------------------------------*/
/* Read sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read(
    BYTE drv,     /* Physical drive number (0) */
    BYTE *buff,   /* Pointer to the data buffer to store read data */
    DWORD sector, /* Start sector number (LBA) */
    UINT count    /* Number of sectors to read (1..128) */
)
{
    if (drv || !count)
        return RES_PARERR; /* Check parameter */
    if (Stat & STA_NOINIT)
        return RES_NOTRDY; /* Check if drive is ready */

    if (!(CardType & CT_BLOCK))
        sector *= 512; /* LBA ot BA conversion (byte addressing cards) */

    if (count == 1)
    {                                      /* Single sector read */
        if ((send_cmd(CMD17, sector) == 0) /* READ_SINGLE_BLOCK */
            && rcvr_datablock(buff, 512))
        {
            count = 0;
        }
    }
    else
    { /* Multiple sector read */
        if (send_cmd(CMD18, sector) == 0)
        { /* READ_MULTIPLE_BLOCK */
            do
            {
                if (!rcvr_datablock(buff, 512))
                    break;
                buff += 512;
            } while (--count);
            send_cmd(CMD12, 0); /* STOP_TRANSMISSION */
        }
    }
    deselect();

    return count ? RES_ERROR : RES_OK; /* Return result */
}

/*-----------------------------------------------------------------------*/
/* Write sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0
DRESULT disk_write(
    BYTE drv,         /* Physical drive number (0) */
    const BYTE *buff, /* Ponter to the data to write */
    DWORD sector,     /* Start sector number (LBA) */
    UINT count        /* Number of sectors to write (1..128) */
)
{
    if (drv || !count)
        return RES_PARERR; /* Check parameter */
    if (Stat & STA_NOINIT)
        return RES_NOTRDY; /* Check drive status */
    if (Stat & STA_PROTECT)
        return RES_WRPRT; /* Check write protect */

    if (!(CardType & CT_BLOCK))
        sector *= 512; /* LBA ==> BA conversion (byte addressing cards) */

    if (count == 1)
    {                                      /* Single sector write */
        if ((send_cmd(CMD24, sector) == 0) /* WRITE_BLOCK */
            && xmit_datablock(buff, 0xFE))
        {
            count = 0;
        }
    }
    else
    { /* Multiple sector write */
        if (CardType & CT_SDC)
            send_cmd(ACMD23, count); /* Predefine number of sectors */
        if (send_cmd(CMD25, sector) == 0)
        { /* WRITE_MULTIPLE_BLOCK */
            do
            {
                if (!xmit_datablock(buff, 0xFC))
                    break;
                buff += 512;
            } while (--count);
            if (!xmit_datablock(0, 0xFD))
                count = 1; /* STOP_TRAN token */
        }
    }
    deselect();

    return count ? RES_ERROR : RES_OK; /* Return result */
}


#endif

/*-----------------------------------------------------------------------*/
/* Miscellaneous drive controls other than data read/write               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl(
	BYTE drv,  /* Physical drive number (0) */
	BYTE cmd,  /* Control command code */
	void *buff /* Pointer to the conrtol data */
)
{
	DRESULT res;
	BYTE n, csd[16];
	DWORD *dp, st, ed, csize;

	if (drv)
		return RES_PARERR; /* Check parameter */
	if (Stat & STA_NOINIT)
		return RES_NOTRDY; /* Check if drive is ready */

	res = RES_ERROR;

	switch (cmd)
	{
	case CTRL_SYNC: /* Wait for end of internal write process of the drive */
		if (select())
			res = RES_OK;
		break;

	case GET_SECTOR_COUNT: /* Get drive capacity in unit of sector (DWORD) */
		if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16))
		{
			if ((csd[0] >> 6) == 1)
			{ /* SDC ver 2.00 */
				csize = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
				*(DWORD *)buff = csize << 10;
			}
			else
			{ /* SDC ver 1.XX or MMC ver 3 */
				n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
				csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
				*(DWORD *)buff = csize << (n - 9);
			}
			res = RES_OK;
		}
		break;

	case GET_BLOCK_SIZE: /* Get erase block size in unit of sector (DWORD) */
		if (CardType & CT_SD2)
		{ /* SDC ver 2.00 */
			if (send_cmd(ACMD13, 0) == 0)
			{ /* Read SD status */
				xchg_spi(0xFF);
				if (rcvr_datablock(csd, 16))
				{ /* Read partial block */
					for (n = 64 - 16; n; n--)
						xchg_spi(0xFF); /* Purge trailing data */
					*(DWORD *)buff = 16UL << (csd[10] >> 4);
					res = RES_OK;
				}
			}
		}
		else
		{ /* SDC ver 1.XX or MMC */
			if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16))
			{ /* Read CSD */
				if (CardType & CT_SD1)
				{ /* SDC ver 1.XX */
					*(DWORD *)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
				}
				else
				{ /* MMC */
					*(DWORD *)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
				}
				res = RES_OK;
			}
		}
		break;

	case CTRL_TRIM: /* Erase a block of sectors (used when _USE_ERASE == 1) */
		if (!(CardType & CT_SDC))
			break; /* Check if the card is SDC */
		if (disk_ioctl(drv, MMC_GET_CSD, csd))
			break; /* Get CSD */
		if (!(csd[0] >> 6) && !(csd[10] & 0x40))
			break; /* Check if sector erase can be applied to the card */
		dp = buff;
		st = dp[0];
		ed = dp[1]; /* Load sector block */
		if (!(CardType & CT_BLOCK))
		{
			st *= 512;
			ed *= 512;
		}
		if (send_cmd(CMD32, st) == 0 && send_cmd(CMD33, ed) == 0 && send_cmd(CMD38, 0) == 0 && wait_ready(30000))
		{				  /* Erase sector block */
			res = RES_OK; /* FatFs does not check result of this command */
		}
		break;

	default:
		res = RES_PARERR;
	}

	deselect();

	return res;
}


/*---------------------------------------------------------*/
/* User provided RTC function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called back     */
/* from FatFs module.                                      */

#if !FF_FS_NORTC && !FF_FS_READONLY
DWORD get_fattime (void)
{
	RTCTIME rtc;

	/* Get local time */
	if (!rtc_gettime(&rtc)) return 0;

	/* Pack date and time into a DWORD variable */
	return	  ((DWORD)(rtc.year - 1980) << 25)
			| ((DWORD)rtc.month << 21)
			| ((DWORD)rtc.mday << 16)
			| ((DWORD)rtc.hour << 11)
			| ((DWORD)rtc.min << 5)
			| ((DWORD)rtc.sec >> 1);
}
#endif