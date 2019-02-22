

#include "pu_uart_driver.h"
#include "led.h"

uart_handle_t *g_uart_handle;

/*=====================================================================
 *
 * 												CONFIGURATION FUNCTIONS
 *
 *======================================================================*/

/**
 *  @brief Enable the given USART peripheral
 *  @param *uartx: Base address of the USART or UART peripheral
 *  @retval None
 */

void pu_uart_enable(USART_TypeDef *uartx)
{
    uartx->CR1 |= USART_REG_C1_USART_EN;
}
/**
 *  @brief Disable the given USART peripheral
 *  @param *uartx: Base address of the USART or UART peripheral
 *  @retval None
 */
void pu_uart_disable(USART_TypeDef *uartx)
{
    uartx->CR1 &= ~USART_REG_C1_USART_EN;
}
/**
 *  @brief Enable/Disable TX Line (Transmiter)
 *  @param *uartx: Base address of the USART or UART peripheral
 *  @param  enable_tx : if enable_rx = 1, enable the RX line, otherwise disable it
 *  @retval None
 */

void pu_uart_enable_disable_tx(USART_TypeDef *uartx, uint32_t enable_tx)
{
    if (enable_tx)
    {
        uartx->CR1 |= USART_REG_CR1_TE;
    }
    else
    {
        uartx->CR1 &= ~USART_REG_CR1_TE;
    }
}
/**
 *  @brief Enable/Disable RX Line (Reciever)
 *  @param *uartx: Base address of the USART or UART peripheral
 *  @param  enable_rx : if enable_rx = 1, enable the RX line, otherwise disable it
 *  @retval None
 */
void pu_uart_enable_disable_rx(USART_TypeDef *uartx, uint32_t enable_rx)
{
    if (enable_rx)
    {
        uartx->CR1 |= USART_REG_CR1_RE;
    }
    else
    {
        uartx->CR1 &= ~USART_REG_CR1_RE;
    }
}
/**
 *  @brief  Configure the word lenght of uart
 *  @param  *uartx: Base address of the USART or UART peripheral
 *  @param  enable_rx : if enable_rx = 1, enable the RX line, otherwise disable it
 *  @retval None
 */
void pu_uart_configure_word_length(USART_TypeDef *uartx, uint32_t word_lenght)
{
    if (word_lenght)
    {
        uartx->CR1 |= USART_REG_CR1_UART_WL; //9 data bits
    }
    else
    {
        uartx->CR1 &= ~USART_REG_CR1_UART_WL; //8 data bits
    }
}

void pu_uart_configure_stop_bits(USART_TypeDef *uartx, uint32_t nstop)
{
    uartx->CR2 &= ~(0x3 << USART_REG_CR2_STOP_BITS);

    if (nstop == USART_STOP_BITS_HALF)
    {
        uartx->CR2 |= (0x01 << USART_REG_CR2_STOP_BITS); //0.5 stop bits
    }
    else if (nstop == USART_STOP_BITS_2) //2 stop bits
    {
        uartx->CR2 |= (0x02 << USART_REG_CR2_STOP_BITS);
    }
    else if (nstop == USART_STOP_BITS_1NHALF)
    {
        uartx->CR2 |= (0x03 << USART_REG_CR2_STOP_BITS); //1.5 stop bits
    }
    else
    {
        uartx->CR2 |= (0x00 << USART_REG_CR2_STOP_BITS); //1 stop bits
    }
}
/**
 *  @brief Program the given baudrate
 *  @param *uartx : Base address of the USART or UART peripheral
 *  @param baud : baudrate value to be programed
 *  @retval None
 */
void pu_uart_set_baud_rate(USART_TypeDef *uartx, uint32_t baud)
{
    uint32_t value;
    if (baud == USART_BAUD_9600)
    {
        value = 0x683;
    }
    else if (baud == USART_BAUD_115200)
    {
        value = 0x8A;
    }
    else
    {
        value = 0x8A;
    }
    uartx->BRR = value;
}

/**
 *  @brief Configure the over sampling rate of the USART peripheral
 *  @param *uartx : Base address of the USART or UART peripheral
 *  @param over8 : if over8=1, then oversampling by 8 will be used, otherwise default 
 *  @retval None
 */
void pu_uart_configure_over_sampling(USART_TypeDef *uartx, uint32_t over8)
{
    if (over8)
    {
        uartx->CR1 |= USART_REG_CR1_OVER8;
    }
}
/**
 *  @brief Enable/Disable the RXNE interrupt
 *  @param *uartx : Base address of the USART or UART peripheral
 *  @param rxne_en : if rxne_en=1, then enable the RXNE interrupt
 *  @retval None
 */
void pu_uart_configure_rxne_interrupt(USART_TypeDef *uartx, uint32_t rxne_en)
{

    if (rxne_en)
    {

        uartx->CR1 |= USART_REG_CR1_RXNE_INT_EN;
    }
    else
    {
        uartx->CR1 &= ~USART_REG_CR1_RXNE_INT_EN;
    }
}

void pu_uart_configure_txe_interrupt(USART_TypeDef *uartx, uint32_t txe_en)
{

    if (txe_en)
    {
        led_turn_on(GPIOD, LED_ORANGE);
        uartx->CR1 |= USART_REG_CR1_TXE_INT_EN;
    }
    else
    {
        uartx->CR1 &= ~USART_REG_CR1_TXE_INT_EN;
    }
}
/**
 *  @brief Enable/Disable the Parity Error interrupt 
 *  @param *uartx : Base address of the USART or UART peripheral
 *  @param pe_en : if pe_en = 1, then enable the parity error interrupt
 *  @retval None
 */
void pu_uart_configure_parity_error_interrupt(USART_TypeDef *uartx, uint32_t pe_en)
{
    if (pe_en)
        uartx->CR1 |= USART_REG_CR1_PEIE_INT_EN;
    else
        uartx->CR1 &= ~USART_REG_CR1_PEIE_INT_EN;
}

/**
 *  @brief Enable/Disable the Error interrupt (Frame error, noise error, overrun error)
 *  @param *uartx : Base address of the USART or UART peripheral
 *  @param er_en : if er_en = 1, then enable the error interrupt
 *  @retval None
 */

void pu_uart_configure_error_interrupt(USART_TypeDef *uartx, uint32_t er_en)
{
    if (er_en)
        uartx->CR3 |= USART_REG_CR3_ERR_INT_EN;
    else
        uartx->CR3 &= ~USART_REG_CR3_ERR_INT_EN;
}
/**
 *  @brief Clear the error flag
 *  @param *huart : pointer to the uart_handle_t structure that 
 *                  contains the configuration ingormatin for the specified UART module
 *  @retval None
 */
void pu_uart_clear_error_flag(uart_handle_t *huart)
{
    uint32_t tmpreg = 0x00;
    tmpreg = huart->Instance->SR;
    tmpreg = huart->Instance->DR;
}

void pu_uart_init(uart_handle_t *uart_handle)
{
    pu_uart_configure_word_length(uart_handle->Instance, uart_handle->Init.WordLength);
    pu_uart_configure_stop_bits(uart_handle->Instance, uart_handle->Init.StopBits);
    pu_uart_configure_over_sampling(uart_handle->Instance, uart_handle->Init.OverSampling);
    pu_uart_set_baud_rate(uart_handle->Instance, uart_handle->Init.BaudRate);
    pu_uart_enable_disable_tx(uart_handle->Instance, uart_handle->Init.Mode);
    pu_uart_enable_disable_rx(uart_handle->Instance, uart_handle->Init.Mode);

    uart_handle->tx_state = PU_UART_STATE_READY;
    uart_handle->rx_state = PU_UART_STATE_READY;
    uart_handle->ErrorCode = PU_UART_ERROR_NONE;

    pu_uart_enable(uart_handle->Instance);
    g_uart_handle = uart_handle;
}

/*=====================================================================
 *
 * 											CALLBACK FUNCTIONS
 *
 *======================================================================*/

/**
 *  @brief UART error callbacks
 *  @param *huart : pointer to the uart_handle_t structure that 
 *                  contains the configuration ingormatin for the specified UART module
 *  @retval None
 */
void pu_uart_error_cb(uart_handle_t *huart)
{
    uint32_t i = 0;
    while (1)
    {

        led_toggle(GPIOD, LED_RED);
        for (i = 0; i < 500000; i++)
            ;
    }
}

/*=====================================================================
 *
 * 												MAIN FUNCTIONS
 *
 *======================================================================*/

/**
  * @brief  Sends an amount of data
	* @param  *uart_handle : pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @param  *buffer:  Pointer to data buffer
  * @param  Size Amount of data to be received
  */
void pu_uart_transmit(uart_handle_t *uart_handle, uint8_t *buffer, uint32_t len)
{

    /*Populate the application given informaions in to the UART handle structure*/
    uart_handle->pTxBuffPtr = buffer;
    uart_handle->TxXferCount = len;
    uart_handle->TxXferSize = len;

    /* This handle is busy in doing the TX */
    uart_handle->tx_state = PU_UART_STATE_BUSY_TX;
    /*Enable the UART periphiral*/
    pu_uart_enable(uart_handle->Instance);

    /*Enable TXE interrupt*/
    pu_uart_configure_txe_interrupt(uart_handle->Instance, 1);
}

/**
  * @brief  Receives an amount of data
	* @param  *uart_handle : pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @param  *buffer:  Pointer to data buffer
  * @param  Size Amount of data to be received
  */
void pu_uart_recieve(uart_handle_t *uart_handle, uint8_t *buffer, uint32_t len)
{
    uint32_t value;
    /*Populate the application given informaions in to the UART handle structure*/
    uart_handle->pRxBuffPtr = buffer;
    uart_handle->RxXferCount = len;
    uart_handle->RxXferSize = len;

    /*This handle is bussy doing the RX*/
    uart_handle->rx_state = PU_UART_STATE_BUSY_RX;

    /*Enable the UART Parity Error Interrupt*/
    pu_uart_configure_parity_error_interrupt(uart_handle->Instance, 1);
    /*Enable the UART Error Interrupt: (Frame error, noise error, overrun error)*/
    pu_uart_configure_error_interrupt(uart_handle->Instance, 1);

    pu_uart_configure_rxne_interrupt(uart_handle->Instance, 1);
}

/*=====================================================================
 *
 * 												HANDLERS
 *
 *======================================================================*/

/**
  * @brief  Handle the transmit interrupt. Called from the USARTx_IRQHandler
	* @param  *uart_handle : pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  */
void pu_uart_handle_TXE_interrupt(uart_handle_t *huart)
{
    uint32_t tmp1 = 0;
    uint8_t val;

    tmp1 = huart->tx_state;
    if (tmp1 == PU_UART_STATE_BUSY_TX)
    {
        val = (uint8_t)(*huart->pTxBuffPtr++ & (uint8_t)0x00FF);
        huart->Instance->DR = val;

        if (--huart->TxXferCount == 0)
        {
            /*Disable the UART TXE Interrupt */
            huart->Instance->CR1 &= ~USART_REG_CR1_TXE_INT_EN;

            /*Enable teh UART Transmit Complete Interrupt */
            huart->Instance->CR1 |= USART_REG_CR1_TCIE_INT_EN;
        }
    }
}

void pu_uart_handle_RXNE_interrupt(uart_handle_t *huart)
{
    uint32_t tmp1 = 0;
    tmp1 = huart->rx_state;
    if (tmp1 == PU_UART_STATE_BUSY_RX)
    {
        //Is application using parity ?
        if (huart->Init.Parity == UART_PARITY_NONE)
        {
            //no parity
            *huart->pRxBuffPtr++ = (uint8_t)(huart->Instance->DR & (uint8_t)0x00FF);
        }
        else
        {
            //Yess, don't ready the most significant bit, because it's a parity bit
            *huart->pRxBuffPtr++ = (uint8_t)(huart->Instance->DR & (uint8_t)0x007F);
        }
        /* are we done with the reception ??*/
        if (--huart->RxXferCount == 0)
        {
            //Yes, disable the RXNE interrupt
            huart->Instance->CR1 &= ~USART_REG_CR1_RXNE_INT_EN;

            /* Disable the UART Parity Error Interrupt*/
            huart->Instance->CR1 &= ~USART_REG_CR1_PEIE_INT_EN;

            /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
            huart->Instance->CR3 &= ~USART_REG_CR3_ERR_INT_EN;

            /* make the state ready for this handle */
            huart->rx_state = PU_UART_STATE_READY;
            /* Call the application callback */
            if (huart->rx_cmp_cb)
                huart->rx_cmp_cb(&huart->RxXferSize);
        }
    }
}

/**
 *  @brief Handle the Transmission Complete (TC) interrupt
 *  @param *huart : pointer to the uart_handle_t structure that
 *                  contains the configuration ingormatin for the specified UART module
 *  @retval none
 */

void pu_uart_handle_TC_interrupt(uart_handle_t *huart)
{
    /* Disable the UART Transmit Complete Interrupt*/
    huart->Instance->CR1 &= ~USART_REG_CR1_TCIE_INT_EN;
    huart->tx_state = PU_UART_STATE_READY;
    /* call the application callback */
    if (huart->tx_cmp_cb)
        huart->tx_cmp_cb(&huart->TxXferSize);
}

/**
 *  @brief Handle the RXNE interrupt
 *  @param *huart : pointer to a uart_handle_t structure that
 *                  contains the configuration ingormatin for the specified UART module
 *  @retval None
 */
void USART2_IRQHandler()
{
    uint32_t tmp1 = 0;
    uint32_t tmp2 = 0;
    tmp1 = g_uart_handle->Instance->SR & USART_REG_SR_PE_FLAG;
    tmp2 = g_uart_handle->Instance->CR1 & USART_REG_CR1_PEIE_INT_EN;
    /* UART parity error interrupt occurred -----------------------------------------*/
    if ((tmp1) && (tmp2))
    {
        pu_uart_clear_error_flag(g_uart_handle);

        g_uart_handle->ErrorCode |= PU_UART_ERROR_PE;
    }
    tmp1 = g_uart_handle->Instance->SR & USART_REG_SR_FE_FLAG;
    tmp2 = g_uart_handle->Instance->CR3 & USART_REG_CR3_ERR_INT_EN;
    /* UART frame error interrupt occurred -----------------------------------------*/
    if ((tmp1) && (tmp2))
    {
        pu_uart_clear_error_flag(g_uart_handle);

        g_uart_handle->ErrorCode |= PU_UART_ERROR_FE;
    }

    tmp1 = g_uart_handle->Instance->SR & USART_REG_SR_RXNE_FLAG;
    tmp2 = g_uart_handle->Instance->CR1 & USART_REG_CR1_RXNE_INT_EN;
    /* UART in mode Reciever -----------------------------------------*/
    if ((tmp1) && (tmp2))
    {
        pu_uart_handle_RXNE_interrupt(g_uart_handle);
    }

    tmp1 = g_uart_handle->Instance->SR & USART_REG_SR_TXE_FLAG;
    tmp2 = g_uart_handle->Instance->CR1 & USART_REG_CR1_TXE_INT_EN;
    /* UART in mode Transmitter -----------------------------------------*/
    if ((tmp1) && (tmp2))
    {

        pu_uart_handle_TXE_interrupt(g_uart_handle);
    }

    tmp1 = g_uart_handle->Instance->SR & USART_REG_SR_TC_FLAG;
    tmp2 = g_uart_handle->Instance->CR1 & USART_REG_CR1_TCIE_INT_EN;
    /* UART in mode Transmitter END -----------------------------------------*/
    if ((tmp1) && (tmp2))
    {
        pu_uart_handle_TC_interrupt(g_uart_handle);
    }

    /* UART frame error interrupt occurred -----------------------------------------*/

    if (g_uart_handle->ErrorCode != PU_UART_ERROR_NONE)
    {
        g_uart_handle->tx_state = PU_UART_STATE_READY;
        g_uart_handle->rx_state = PU_UART_STATE_READY;

        pu_uart_error_cb(g_uart_handle);
    }
}
