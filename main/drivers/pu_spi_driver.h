

//         | Pins pack 1  |       |       | Pins pack 2  |       |       | Pins pack 3  |       |      |
//  -------|--------------|-------|-------|--------------|-------|-------|--------------|-------|------|-----
//   SPIx  | MOSI         | MISO  | SCK   | MOSI         | MISO  | SCK   | MOSI         | MISO  | SCK  | APB
//   SPI1  | PA7          | PA6   | PA5   | PB5          | PB4   | PB3   |              |       |      | 2
//   SPI2  | PC3          | PC2   | PB10  | PB15         | PB14  | PB13  | PI3          | PI2   | PI0  | 1
//   SPI3  | PB5          | PB4   | PB3   | PC12         | PC11  | PC10  |              |       |      | 1
//   SPI4  | PE6          | PE5   | PE2   | PE14         | PE13  | PE12  |              |       |      | 2
//   SPI5  | PF9          | PF8   | PF7   | PF11         | PH7   | PH6   |              |       |      | 2
//   SPI6  | PG14         | PG12  | PG13  |              |       |       |              |       |      | 2

#ifndef __PU_SPI_DRIVER_H
#define __PU_SPI_DRIVER_H

#include "stm32f407xx.h"

/******************************** BIT defintions for SPI_CR1 register ****************************************/

#define PU_SPI_REG_CR1_BIDIMODE ((uint32_t)1 << 15)
#define PU_SPI_ENABLE_2_LINE_UNI_DIR 0
#define PU_SPI_ENABLE_1_LINE_BIDI 1

#define PU_SPI_REG_CR1_DFF ((uint32_t)1 << 11)
#define PU_SPI_8BIT_DF 0
#define PU_SPI_16BIT_DF 1

#define PU_SPI_REG_CR1_SSM ((uint32_t)1 << 9)
#define PU_SPI_SSM_ENABLE 0
#define PU_SPI_SSM_DISABLE 1

#define PU_SPI_REG_CR1_SSI ((uint32_t)1 << 8)

#define PU_SPI_CR1_LSB_FIRST ((uint32_t)1 << 7)
#define PU_SPI_TX_MSB_FIRST 0
#define PU_SPI_TX_LSB_FIRST 1

#define PU_SPI_REG_CR1_SPE ((uint32_t)1 << 6) //SPI peripheral enable

#define PU_SPI_REG_CR1_BR_PCLK_DIV_2 ((uint32_t)0 << 3)
#define PU_SPI_REG_CR1_BR_PCLK_DIV_4 ((uint32_t)1 << 3)
#define PU_SPI_REG_CR1_BR_PCLK_DIV_8 ((uint32_t)2 << 3)
#define PU_SPI_REG_CR1_BR_PCLK_DIV_16 ((uint32_t)3 << 3)
#define PU_SPI_REG_CR1_BR_PCLK_DIV_32 ((uint32_t)4 << 3)
#define PU_SPI_REG_CR1_BR_PCLK_DIV_64 ((uint32_t)5 << 3)
#define PU_SPI_REG_CR1_BR_PCLK_DIV_128 ((uint32_t)6 << 3)
#define PU_SPI_REG_CR1_BR_PCLK_DIV_256 ((uint32_t)7 << 3)

#define PU_SPI_REG_CR1_MSTR ((uint32_t)1 << 2)
#define PU_SPI_MASTER_MODE_SEL 1
#define PU_SPI_SLAVE_MODE_SEL 0

#define PU_SPI_REG_CR1_CPOL ((uint32_t)1 << 1)

#define PU_SPI_CPOL_LOW 0
#define PU_SPI_CPOL_HIGH 1

#define PU_SPI_REG_CR1_CPHA ((uint32_t)1 << 0)
#define PU_SPI_FIRST_CLOCK_TRANS 0
#define PU_SPI_SECOND_CLOCK_TRANS 1

/****************** Bit definition for SPI_CR2 register ***********************/
#define PU_SPI_REG_CR2_TXEIE_ENABLE ((uint32_t)1 << 7)
#define PU_SPI_REG_CR2_RXNEIE_ENABLE ((uint32_t)1 << 6)
#define PU_SPI_REG_CR2_ERRIE_ENABLE ((uint32_t)1 << 5)

#define PU_SPI_REG_CR2_FRAME_FORMAT ((uint32_t)1 << 4)
#define PU_SPI_MOTOROLA_MODE 0
#define PU_SPI_TI_MODE 1

#define PU_SPI_REG_CR2_SSOE ((uint32_t)1 << 2)

/****************** Bit definition for SPI_SR register ***********************/
#define PU_SPI_REG_SR_FRE_FLAG ((uint32_t)1 << 8)
#define PU_SPI_REG_SR_BUSY_FLAG ((uint32_t)1 << 7)
#define PU_SPI_REG_SR_TXE_FLAG ((uint32_t)1 << 1)
#define PU_SPI_REG_SR_RXNE_FLAG ((uint32_t)1 << 0)

/*****SPI device base address */
#define SPI_1 SPI1
#define SPI_2 SPI2
#define SPI_3 SPI3

#define PU_SPI_IS_BUSY 1
#define PU_SPI_IS_NOT_BUSY 0

/* Macros to Enable Clock for different SPI devices */
#define _PU_RCC_SPI1_CLK_ENABLE() (RCC->APB2ENR |= (1 << 12))
#define _PU_RCC_SPI2_CLK_ENABLE() (RCC->APB1ENR |= (1 << 14))
#define _PU_RCC_SPI3_CLK_ENABLE() (RCC->APB1ENR |= (1 << 15))

#define RESET 0
#define SET !RESET

/****************************************************************************/
/*																																					*/
/*											2. Data structures used by SPI Driver								*/
/*																																					*/
/****************************************************************************/
typedef enum
{
	PU_SPI_STATE_RESET = 0x00,
	PU_SPI_STATE_READY = 0x01,
	PU_SPI_STATE_BUSY = 0x02,
	PU_SPI_STATE_BUSY_TX = 0x12,
	PU_SPI_STATE_BUSY_RX = 0x22,
	PU_SPI_STATE_BUSY_TX_RX = 0x32,
	PU_SPI_STATE_ERROR = 0x03
} pu_spi_state_t;

typedef struct
{
	uint32_t Mode;
	uint32_t Direction;
	uint32_t DataSize;
	uint32_t CLKPolarity;
	uint32_t CLKPhase;
	uint32_t NSS;
	uint32_t BaudRatePrescaler;
	uint32_t FirstBit;
} pu_spi_init_t;

typedef struct __spi_handle_t
{
	SPI_TypeDef *Instance;

	pu_spi_init_t Init;

	uint8_t *pTxBuffPtr;

	uint16_t TxXferSize;

	uint16_t TxXferCount;

	uint8_t *pRxBuffPtr;

	uint16_t RxXferSize;

	uint16_t RxXferCount;

	pu_spi_state_t State;
} pu_spi_handle_t;

/****************************************************************************/
/*																																					*/
/*											3. Driver exposed API's															*/
/*																																					*/
/****************************************************************************/

void pu_spi_init(pu_spi_handle_t *spi_handle);

void pu_spi_master_tx(pu_spi_handle_t *spi_handle, uint8_t *buffer, uint32_t len);
void pu_spi_slave_rx(pu_spi_handle_t *spi_handle, uint8_t *buffer, uint32_t len);
void pu_spi_slave_tx(pu_spi_handle_t *spi_handle, uint8_t *rcv_buffer, uint32_t len);

void pu_spi_irq_handler(pu_spi_handle_t *handler);

void pu_spi_handle_tx_interrupt(pu_spi_handle_t *handler);
void pu_spi_handle_rx_interrupt(pu_spi_handle_t *handler);

static void pu_spi_configure_baudrate(SPI_TypeDef *SPIx, uint32_t pre_scalar_value);
static void pu_spi_configure_datasize_direction(SPI_TypeDef *SPIx, uint32_t datasize_16, uint32_t lsbfirst);
static void pu_spi_configure_direction(SPI_TypeDef *SPIx, uint32_t direction);
static void pu_spi_configure_datasize(SPI_TypeDef *SPIx, uint32_t datasize_16, uint32_t lsbfirst);
static void pu_spi_configure_nss_master(SPI_TypeDef *SPIx, uint32_t ssm_enable);
static void pu_spi_configure_nss_slave(SPI_TypeDef *SPIx, uint32_t ssm_enable);
#endif
