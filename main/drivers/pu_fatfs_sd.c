/*-----------------------------------------------------------------------*/
/* SPI controls (Platform dependent)                                     */
/*-----------------------------------------------------------------------*/

#include "pu_fatfs_sd.h"
#include "pu_spi_driver.h"
#include "pu_gpio_driver.h"
#include "tm_stm32_delay.h"
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

static volatile DSTATUS Stata = STA_NOINIT; /* Physical drive status */
static volatile UINT Timer1, Timer2;        /* 1kHz decrement timer stopped at zero (disk_timerproc()) */
static BYTE CardType;                       /* Card type flags */

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
    /* Configure GPIOB_PIN_5 for spi CLK functionality */
    spi_conf.pin = SPI_CLK_PIN;
    spi_conf.mode = GPIO_PIN_ALT_FUN_MODE;
    spi_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
    spi_conf.pull = GPIO_PIN_PULL_DOWN;
    spi_conf.speed = GPIO_PIN_SPEED_MEDIUM;
    pu_gpio_set_alt_function(GPIOB, PU_FATFS_SPI_CLK_PIN, GPIO_PIN_AF5_SPI2);
    pu_gpio_init(GPIOB, &spi_conf);

    /* configure GPIOB_PIN_5 for SPI MOSI functionality */
    spi_conf.pin = PU_FATFS_SPI_MOSI_PIN;
    spi_conf.pull = GPIO_PIN_PULL_UP;
    pu_gpio_set_alt_function(GPIOB, PU_FATFS_SPI_MOSI_PIN, GPIO_PIN_AF5_SPI2);
    pu_gpio_init(GPIOB, &spi_conf);

    /* configure GPIOB_PIN_4 for SPI MISO functionality */
    spi_conf.pin = PU_FATFS_SPI_MISO_PIN;
    spi_conf.pull = GPIO_PIN_PULL_UP;
    pu_gpio_set_alt_function(GPIOB, PU_FATFS_SPI_MISO_PIN, GPIO_PIN_AF5_SPI2);
    pu_gpio_init(GPIOB, &spi_conf);
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

    pu_spi_init(&pu_fatfs_spi_handle);

    NVIC_EnableIRQ(SPI2_IRQn);

    PU_FATFS_CS_HIGH(); /* Set CS pin High */
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

    pu_spi_master_tx(&pu_fatfs_spi_handle, data, 8);
    while (&pu_fatfs_spi_handle.State != PU_SPI_STATE_READY)
        ;
    delay_gen();
    pu_spi_master_rx(&pu_fatfs_spi_handle, ack_buf, 2);
    return (BYTE)*pu_fatfs_spi_handle->Instance->DR;
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
    PU_FATFS_CS_HIGH;             /* CS = H */
    TM_SPI_Send(FATFS_SPI, 0xFF); /* Dummy clock (force DO hi-z for multiple slave SPI) */
    // FATFS_DEBUG_SEND_USART("deselect: ok");
}

/**
	* @brief  Select card and wait for ready
	* @param  None
	* @retval None
	*/
static void select(void)
{
    FATFS_CS_LOW;
}