
#include "pu_spi_driver.h"

static void pu_spi_configure_device_mode(SPI_TypeDef *SPIx, uint32_t master)
{

	if (master)
	{
		SPIx->CR1 |= PU_SPI_REG_CR1_MSTR;
	}
	else
	{
		SPIx->CR1 &= ~PU_SPI_REG_CR1_MSTR;
	}
}
/**
	* @brief  Configures SPI phase and polarity
	* @param  *SPIx : Base address of the SPI  
  	* @param  phase_value : phase value
	* @param  polarity : polarity value
	* @retval None
*/
static void pu_spi_configure_phase_and_polarity(SPI_TypeDef *SPIx, uint32_t phase_value, uint32_t polarity)
{

	if (phase_value)
	{
		SPIx->CR1 |= PU_SPI_REG_CR1_CPHA;
	}
	else
	{
		SPIx->CR1 &= ~PU_SPI_REG_CR1_CPHA;
	}

	if (polarity)
	{
		SPIx->CR1 |= PU_SPI_REG_CR1_CPOL;
	}
	else
	{
		SPIx->CR1 &= ~PU_SPI_REG_CR1_CPOL;
	}
}

/**
	* @brief  Configures SPI datasize 
	* @param  *SPIx : Base address of the SPI  
  	* @param  datasize : data size to be configured  ,  
  	* @param  lsbmsbfirst : if 1, lsb will be sent first.  
	* @retval None
*/
static void pu_spi_configure_datasize(SPI_TypeDef *SPIx, uint32_t datasize_16, uint32_t lsbfirst)
{
	if (datasize_16)
	{
		SPIx->CR1 |= PU_SPI_REG_CR1_DFF;
	}
	else
	{
		SPIx->CR1 &= ~PU_SPI_REG_CR1_DFF;
	}

	if (lsbfirst)
	{
		SPIx->CR1 |= PU_SPI_CR1_LSB_FIRST;
	}
	else
	{
		SPIx->CR1 &= ~PU_SPI_CR1_LSB_FIRST;
	}
}

/**
	* @brief  Configures SPI direction
	* @param  *SPIx : Base address of the SPI  
  	* @param  direction : if 1, direction will be single line bi-directional else, 2 lines uni directional 
	* @retval None
*/
static void pu_spi_configure_direction(SPI_TypeDef *SPIx, uint32_t direction)
{
	if (direction)
	{
		SPIx->CR1 |= PU_SPI_REG_CR1_BIDIMODE;
	}
	else
	{
		SPIx->CR1 &= ~PU_SPI_REG_CR1_BIDIMODE;
	}
}

/**
	* @brief  Configures SPI baudrate
	* @param  *SPIx : Base address of the SPI  
  	* @param  pre_scalar_value : pre scalar value to be used to generate baudrate 
	* @retval None
	*/
static void pu_spi_configure_baudrate(SPI_TypeDef *SPIx, uint32_t pre_scalar_value)
{
	if (pre_scalar_value > 7)
		SPIx->CR1 |= (0x00 << 3); //if pre_scalar_value > 7,then use default . that is 0
	else
		SPIx->CR1 |= (pre_scalar_value << 3);
}

static void pu_spi_configure_datasize_direction(SPI_TypeDef *SPIx, uint32_t datasize_16, uint32_t lsbfirst)
{

	if (datasize_16)
	{
		SPIx->CR1 |= PU_SPI_REG_CR1_DFF; // 16 bit data at a time
	}
	else
	{
		SPIx->CR1 &= ~PU_SPI_REG_CR1_DFF; // 8 bit data at a time
	}

	if (lsbfirst)
	{
		SPIx->CR1 |= PU_SPI_CR1_LSB_FIRST; // send LSB first
	}
	else
	{
		SPIx->CR1 &= ~PU_SPI_CR1_LSB_FIRST; // send MSB first
	}
}

/**
 *  @brief Configure the select master pin
 *  @param *uartx: Base address of the USART or UART peripheral
 *  @param  enable_tx : if enable_rx = 1, enable the RX line, otherwise disable it
 *  @retval None
 */

static void pu_spi_configure_nss_master(SPI_TypeDef *SPIx, uint32_t ssm_enable)
{
	if (ssm_enable)
	{
		SPIx->CR1 |= PU_SPI_REG_CR1_SSM;
		SPIx->CR1 |= PU_SPI_REG_CR1_SSI;
	}
	else
	{
		SPIx->CR1 &= ~PU_SPI_REG_CR1_SSM;
	}
}

/**
 *  @brief Enable/Disable TX Line (Transmiter)
 *  @param *uartx: Base address of the USART or UART peripheral
 *  @param  enable_tx : if enable_rx = 1, enable the RX line, otherwise disable it
 *  @retval None
 */
static void pu_spi_configure_nss_slave(SPI_TypeDef *SPIx, uint32_t ssm_enable)
{
	if (ssm_enable)
	{
		SPIx->CR1 |= PU_SPI_REG_CR1_SSM; //Software slave management mode
	}
	else
	{
		SPIx->CR1 &= ~PU_SPI_REG_CR1_SSM; //Hardware slave management mode
	}
}

/**
 *  @brief Enables the SPI device
*  	@param *SPIx : Base adress of the SPI
 *  @retval None
 */
static void pu_spi_enable(SPI_TypeDef *SPIx)
{
	if (!(SPIx->CR1 & PU_SPI_REG_CR1_SPE))
		SPIx->CR1 |= PU_SPI_REG_CR1_SPE;
}

/**
 *  @brief Disables the SPI device
 *  @param 	*SPIx : Base address of the SPI
 *  @retval None
 */
static void pu_spi_disable(SPI_TypeDef *SPIx)
{
	SPIx->CR1 &= ~PU_SPI_REG_CR1_SPE;
}

/**
 *  @brief 	Enable the Tx buffer empty interrupt (TXE)
 *  @param 	*SPIx : Base address of the SPI
 *  @retval None
 */
void pu_spi_enable_txe_interrupt(SPI_TypeDef *SPIx)
{
	SPIx->CR2 |= PU_SPI_REG_CR2_TXEIE_ENABLE;
}

/**
 *  @brief 	Disable the Tx buffer empty interrupt (TXE)
 *  @param 	*SPIx : Base address of the SPI
 *  @retval None
 */
void pu_spi_disable_txe_interrupt(SPI_TypeDef *SPIx)
{
	SPIx->CR2 &= ~PU_SPI_REG_CR2_TXEIE_ENABLE;
}

/**
 *  @brief 	Enable the RX buffer not empty interrupt (RXNE)
 *  @param 	*SPIx : Base address of the SPI
 *  @retval None
 */
void pu_spi_enable_rxne_interrupt(SPI_TypeDef *SPIx)
{
	SPIx->CR2 |= PU_SPI_REG_CR2_RXNEIE_ENABLE;
}
/**
 *  @brief 	Disable the RX buffer not empty interrupt (RXNE)
 *  @param 	*SPIx : Base address of the SPI
 *  @retval None
 */
void pu_spi_disable_rxne_interrupt(SPI_TypeDef *SPIx)
{
	SPIx->CR2 &= ~PU_SPI_REG_CR2_RXNEIE_ENABLE;
}

/**
 *  @brief 	 Transfer data through master to slave
 *  @param 	*SPIx : Base address of the SPI
 *  @param 	*buffer : pointer to the tx buffer
 *  @param 	 len : length of the tx data
 *  @retval  None
 */
void pu_spi_master_tx(pu_spi_handle_t *spi_handle, uint8_t *buffer, uint32_t len)
{
	spi_handle->pTxBuffPtr = buffer;
	spi_handle->TxXferCount = len;
	spi_handle->TxXferSize = len;

	//Bussy doing TX
	spi_handle->State = PU_SPI_STATE_BUSY_TX;

	pu_spi_enable(spi_handle->Instance);

	/*Tx buffer is empty and since we enabled the interrupt for TXE event you will get an
		immidiate interupt after this code*/
	pu_spi_enable_txe_interrupt(spi_handle->Instance);
}

/**
 *  @brief 	 API used to do master data reception
 *  @param 	*SPIx : Base address of the SPI
 *  @param 	*buffer : pointer to the tx buffer
 *  @param 	 len : length of the rx data
 *  @retval  None
 */

void pu_spi_master_rx(pu_spi_handle_t *spi_handle, uint8_t *rx_buffer, uint32_t len)
{
	uint32_t i = 0, val;

	/* Sending a dummy tx */
	spi_handle->pTxBuffPtr = rx_buffer;
	spi_handle->RxXferSize = len;
	spi_handle->RxXferCount = len;
	/* data will be rxed to rx_buffer */
	spi_handle->pRxBuffPtr = rx_buffer;
	spi_handle->RxXferSize = len;
	spi_handle->RxXferCount = len;

	/* Driver is busy in RX */
	spi_handle->State = PU_SPI_STATE_BUSY_RX;

	pu_spi_enable(spi_handle->Instance);

	/* read data register once before enabling
	*  the RXNE interrupt to make sure DR is empty */
	val = spi_handle->Instance->DR;

	/* Now enable both TXE and RXNE Interrupt */
	pu_spi_enable_rxne_interrupt(spi_handle->Instance);
	pu_spi_enable_txe_interrupt(spi_handle->Instance);
}

/**
 *  @brief 	 API used to do slave data transfer
 *  @param 	*SPIx : Base address of the SPI
 *  @param 	*buffer : pointer to the tx buffer
 *  @param 	 len : length of the tx data
 *  @retval  None
 */
void pu_spi_slave_tx(pu_spi_handle_t *spi_handle, uint8_t *tx_buffer, uint32_t len)
{
	uint32_t i = 0, val;

	/* populate the pointers and length information to TX the data */
	spi_handle->pTxBuffPtr = tx_buffer;
	spi_handle->TxXferSize = len;
	spi_handle->TxXferCount = len;
	/* pointes to handle dummy rx, you can reuse the same pointer */
	spi_handle->pRxBuffPtr = tx_buffer;
	spi_handle->RxXferSize = len;
	spi_handle->RxXferCount = len;

	/* Driver is busy in doing TX */
	spi_handle->State = PU_SPI_STATE_BUSY_TX;

	pu_spi_enable(spi_handle->Instance);

	/* Now enable both TXE and RXNE Interrupt */
	pu_spi_enable_rxne_interrupt(spi_handle->Instance);
	pu_spi_enable_txe_interrupt(spi_handle->Instance);
}

/**
 *  @brief 	 API used to do slave data recieve
 *  @param 	*SPIx : Base address of the SPI
 *  @param 	*rcv_buffer : pointer to the rx buffer
 *  @param 	 len : length of the rx data
 *  @retval  None
 */
void pu_spi_slave_rx(pu_spi_handle_t *spi_handle, uint8_t *rcv_buffer, uint32_t len)
{
	/* populate the rcv buffer pointer address along with size in the handler*/
	spi_handle->pRxBuffPtr = rcv_buffer;
	spi_handle->RxXferSize = len;
	spi_handle->RxXferCount = len;

	/* Driver busy in RX */
	spi_handle->State = PU_SPI_STATE_BUSY_RX;

	/* enable the peripheral, if it's not enabled */
	pu_spi_enable(spi_handle->Instance);

	/* slave need to recieve data, so enable the RXNE interrupt 
		Byte reception will be taken care in the RXNE interrupt handling code
	*/
	pu_spi_enable_rxne_interrupt(spi_handle->Instance);
}
/**
  * @brief  This function handles SPI interrupt request.
  * @param  *handler: pointer to a pu_spi_handle_t structure that contains
  *                the configuration information for SPI module.
  * @retval none
  */
void pu_spi_irq_handler(pu_spi_handle_t *handler)
{

	uint32_t tmp1 = 0;

	tmp1 = (handler->Instance->SR & PU_SPI_REG_SR_RXNE_FLAG);
	if (tmp1 != RESET)
	{
		/*RXNE flag is set handle the RX of data bytes*/
		pu_spi_handle_rx_interrupt(handler);

		return;
	}

	tmp1 = (handler->Instance->SR & PU_SPI_REG_SR_TXE_FLAG);
	if ((tmp1 != RESET))
	{
		/*TXE flag is set, handle the TX of data types*/
		pu_spi_handle_tx_interrupt(handler);
		return;
	}
}

/**
	* @brief  Checks whether bus is free or busy 
	* @param  *SPIx : Base address of the SPI  
  	* @retval return 1, if bus is busy 
	*/
static uint8_t pu_spi_is_bus_busy(SPI_TypeDef *SPIx)
{
	if (SPIx->SR & PU_SPI_REG_SR_BUSY_FLAG)
	{
		return PU_SPI_IS_BUSY; // 1
	}
	else
		return PU_SPI_IS_NOT_BUSY; // 0
}

/**
 *  @brief 	 close TX transfer
 *  @param 	*handler: pointer to a spi_handle_t structure that contains
 * 			the configuration information for SPI module.
 *  @retval  void
 */
static void pu_spi_close_tx_interrupt(pu_spi_handle_t *handler)
{

	pu_spi_disable_txe_interrupt(handler->Instance);

	if (handler->Init.Mode && (handler->State != PU_SPI_STATE_BUSY_RX))
		handler->State = PU_SPI_STATE_READY;
}

/**
 *  @brief 	 close RX Transfer
 *  @param 	*handler: pointer to a spi_handle_t structure that contains
 * 			the configuration information for SPI module.
 *  @retval  void
 */
static void pu_spi_close_rx_interrupt(pu_spi_handle_t *handler)
{

	while (pu_spi_is_bus_busy(handler->Instance))
		;
	pu_spi_disable_rxne_interrupt(handler->Instance);
	handler->State = PU_SPI_STATE_READY;
}

/**
 *  @brief 	 Handle the TXE interrupt
 *  @param 	*handler: pointer to a spi_handle_t structure that contains
 * 			the configuration information for SPI module.
 *  @retval  None
 */
void pu_spi_handle_tx_interrupt(pu_spi_handle_t *handler)
{

	/* Transmit data in 8Bit mode */
	if (handler->Init.DataSize == PU_SPI_8BIT_DF)
	{
		handler->Instance->DR = (*handler->pTxBuffPtr++);
		handler->TxXferCount--; //we sent 1 byte
	}
	else
	{
		handler->Instance->DR = *((uint16_t *)handler->pTxBuffPtr++);
		handler->pTxBuffPtr += 2;
		handler->TxXferCount -= 2; //we send 2bytes in one go
	}
	if (handler->TxXferCount == 0)
	{
		/* End of transmition was reached, close the TXE interrupt */
		pu_spi_close_tx_interrupt(handler);
	}
}

/**
 *  @brief 	 Handle the RXNE interrupt
 *  @param 	*handler: pointer to a spi_handle_t structure that contains
 * 			the configuration information for SPI module.
 *  @retval  None
 */
void pu_spi_handle_rx_interrupt(pu_spi_handle_t *handler)
{

	/* Recieve data in 8Bit mode */
	if (handler->Init.DataSize == PU_SPI_8BIT_DF)
	{
		//Null check
		if (handler->pRxBuffPtr++)
			(*handler->pRxBuffPtr++) = handler->Instance->DR;
		handler->RxXferCount--;
	}
	/* Recieve data in 16bit mode */
	else
	{
		*((uint16_t *)handler->pRxBuffPtr) = handler->Instance->DR;
		handler->pRxBuffPtr += 2;
		handler->RxXferCount -= 2;
	}
	if (handler->TxXferCount == 0)
	{
		/* End of transmition was reached, close the RX interrupt */
		pu_spi_close_rx_interrupt(handler);
	}
}

/**
	 *  @brief 	 Initialize spi
	 *  @param 	*handler: pointer to a spi_handle_t structure that contains
	 * 			the configuration information for SPI module.
	 *  @retval  None
*/
void pu_spi_init(pu_spi_handle_t *handler)
{
	/* configure the phase and polartiy */
	pu_spi_configure_phase_and_polarity(handler->Instance,
										handler->Init.CLKPhase, handler->Init.CLKPolarity);

	/* Configure the spi device mode */
	pu_spi_configure_device_mode(handler->Instance, handler->Init.Mode);

	/* Configure the spi data size */
	pu_spi_configure_datasize(handler->Instance, handler->Init.DataSize, handler->Init.FirstBit);

	/* Configure the slave select line */
	if (handler->Init.Mode == PU_SPI_MASTER_MODE_SEL)
		pu_spi_configure_nss_master(handler->Instance, handler->Init.NSS);
	else
		pu_spi_configure_nss_slave(handler->Instance, handler->Init.NSS);

	/* Configure the  SPI deivce speed */
	pu_spi_configure_baudrate(handler->Instance, handler->Init.BaudRatePrescaler);

	/* Configure the SPI device direction */
	pu_spi_configure_direction(handler->Instance, handler->Init.Direction);
}