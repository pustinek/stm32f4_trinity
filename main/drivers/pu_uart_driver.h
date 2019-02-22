#ifndef __UART_DRIVER_H
#define __UART_DRIVER_H

#include "stm32f4xx.h"

typedef enum
{
    PU_UART_STATE_RESET = 0x00,
    PU_UART_STATE_READY = 0x01,
    PU_UART_STATE_BUSY = 0x02,
    PU_UART_STATE_BUSY_TX = 0x12,
    PU_UART_STATE_BUSY_RX = 0x22,
    PU_UART_STATE_BUSY_TX_RX = 0x32,

} uart_state_t;

// UART possible error codes
#define PU_UART_ERROR_NONE 0x00000000U /*!< No error            */
#define PU_UART_ERROR_PE 0x00000001U   /*!< Parity error        */
#define PU_UART_ERROR_NE 0x00000002U   /*!< Noise error         */
#define PU_UART_ERROR_FE 0x00000004U   /*!< Frame error         */
#define PU_UART_ERROR_ORE 0x00000008U  /*!< Overrun error       */
#define PU_UART_ERROR_DMA 0x00000010U  /*!< DMA transfer error  */

// Different USART and UART peripheral base addresses
#define USART_1 USART1
#define USART_2 USART2
#define USART_3 USART3
#define USART_4 USART4
#define USART_5 USART5
#define USART_6 USART6

// Macros to enable the closk for various UART
#define _PU_RCC_USART1_CLK_ENABLE() (RCC->APB2ENR |= (1 << 4))
#define _PU_RCC_USART2_CLK_ENABLE() (RCC->APB1ENR |= (1 << 17))
#define _PU_RCC_USART3_CLK_ENABLE() (RCC->APB1ENR |= (1 << 18))
#define _PU_RCC_UART4_CLK_ENABLE() (RCC->APB1ENR |= (1 << 19))
#define _PU_RCC_UART5_CLK_ENABLE() (RCC->APB1ENR |= (1 << 20))
#define _PU_RCC_USART6_CLK_ENABLE() (RCC->APB2ENR |= (1 << 5))

// Bit Definition for USART_SR register

#define USART_REG_SR_TXE_FLAG ((uint32_t)(1 << 7))
#define USART_REG_SR_TC_FLAG ((uint32_t)(1 << 6))
#define USART_REG_SR_RXNE_FLAG ((uint32_t)(1 << 5))
#define USART_REG_SR_IDLE_FLAG ((uint32_t)(1 << 4))
#define USART_REG_SR_ORE_FLAG ((uint32_t)(1 << 3))
#define USART_REG_SR_NE_FLAG ((uint32_t)(1 << 2))
#define USART_REG_SR_FE_FLAG ((uint32_t)(1 << 1))
#define USART_REG_SR_PE_FLAG ((uint32_t)(1 << 0))

//Bit defenition for USART_BRR register
#define USART_REG_BRR_MANTISSA ((uint32_t)(1 << 4))
#define USART_REG_BRR_FRACTION ((uint32_t)(1 << 0))

//Bit defintion for USART_CR1 register
#define USART_REG_CR1_OVER8 ((uint32_t)(1 << 15))
#define USART_REG_C1_USART_EN ((uint32_t)(1 << 13))
#define USART_REG_CR1_UART_WL ((uint32_t)(1 << 12))

#define USART_REG_CR1_TXE_INT_EN ((uint32_t)(1 << 7))
#define USART_REG_CR1_TCIE_INT_EN ((uint32_t)(1 << 6))
#define USART_REG_CR1_RXNE_INT_EN ((uint32_t)(1 << 5))
#define USART_REG_CR1_PEIE_INT_EN ((uint32_t)(1 << 8))
#define USART_REG_CR1_TE ((uint32_t)(1 << 3)) //Transmiter enabled
#define USART_REG_CR1_RE ((uint32_t)(1 << 2)) // Reciever enabled

#define USART_REG_CR2_STOP_BITS 12

#define USART_REG_CR3_ERR_INT_EN ((uint32_t)(1 << 0))

// Baud rates
#define USART_BAUD_9600 0x683
#define USART_BAUD_115200 0x8A

#define UART_PARITY_NONE 0

#define UART_STOP_BITS_1 0x00
#define USART_STOP_BITS_HALF 0x01
#define USART_STOP_BITS_2 0x2
#define USART_STOP_BITS_1NHALF 0x3

#define USART_WL_1S8B 0
#define USART_WL_1S9B 1

#define USART_OVER8_ENABLE 1
#define USART_OVER16_ENABLE 0

#define UART_MODE_TX_RX 0x3
/** 
  * @brief UART Init Structure definition  
 */
typedef struct
{
    uint32_t BaudRate; /*This member configures the UART communication baud rate*/

    uint32_t WordLength; /*Specify the number of data bits transmitted or recieved in a frame*/

    uint32_t StopBits; /*Specify the number of stop bits transmited*/

    uint32_t Parity; /*Specify the parity mode*/

    uint32_t Mode; /*Specify whether the Recieve or Transmit mode is enabled or disabled*/

    uint32_t OverSampling; /*Specify whether the Over sampling 8 is enabled or disabled*/
} uart_init_t;

/* Applcation callbacks typedef */
typedef void(TX_COMP_CB_t)(void *ptr);
typedef void(RX_COMP_CB_t)(void *ptr);

/** 
  * @brief  UART handle Structure definition  
  */
typedef struct
{
    USART_TypeDef *Instance; /*!< UART registers base address        */

    uart_init_t Init; /*!< UART communication parameters      */

    uint8_t *pTxBuffPtr; /*!< Pointer to UART Tx transfer Buffer */

    uint16_t TxXferSize; /*!< UART Tx Transfer size              */

    uint16_t TxXferCount; /*!< UART Tx Transfer Counter           */

    uint8_t *pRxBuffPtr; /*!< Pointer to UART Rx transfer Buffer */

    uint16_t RxXferSize; /*!< UART Rx Transfer size              */

    uint16_t RxXferCount; /*!< UART Rx Transfer Counter           */

    uart_state_t rx_state;

    uart_state_t tx_state;

    uint32_t ErrorCode;

    TX_COMP_CB_t *tx_cmp_cb;

    RX_COMP_CB_t *rx_cmp_cb;

} uart_handle_t;

void pu_uart_init(uart_handle_t *handle);

void pu_uart_transmit(uart_handle_t *handle, uint8_t *buffer, uint32_t len);

void pu_uart_recieve(uart_handle_t *handle, uint8_t *buffer, uint32_t len);

void pu_uart_handle_interrupt(uart_handle_t *huart);

#endif
