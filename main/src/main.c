
#include "led.h"
#include <stdio.h>
#include "pu_uart_driver.h"
#include "pu_spi_driver.h"
#include "spi_main.h"
#include "stm32f4xx.h"

uart_handle_t uart_handle2;
pu_spi_handle_t spi_handle;
uint8_t rx_buffer[10];

/*=====================================================	
 *	
 * 					UART STUFF
 * 
 * ==================================================== */

void handle_cmd(int cmd, int led)
{
	if (cmd == 'H')
	{
		if (led == (int)0xff)
		{
		}
		else
		{
			led_turn_on(GPIOD, led);
		}
	}
	else if (cmd == 'L')
	{
		if (led == (int)0xff)
		{
		}
		else
		{
			led_turn_off(GPIOD, led);
		}
	}
}

void parse_uart_commands(uint8_t *cmd)
{
	if (cmd[0] == 'L' && cmd[1] == 'E' && cmd[2] == 'D')
	{
		if (cmd[3] == 'O')
		{
			handle_cmd(cmd[4], LED_ORANGE);
		}
		else if (cmd[3] == 'B')
		{
			handle_cmd(cmd[4], LED_BLUE);
		}
	}
}

void app_tx_cmp_callback(void *size)
{
}

void app_rx_cmp_callback(void *size)
{
	parse_uart_commands(rx_buffer);
}

void uart_gpio_init(void)
{

	gpio_pin_conf_t uart_pin_conf;

	_HAL_RCC_GPIOA_CLK_ENABLE();

	/* GPIO_PORT_A_PIN_2 will be used as TX */
	uart_pin_conf.pin = 2;
	uart_pin_conf.mode = GPIO_PIN_ALT_FUN_MODE;
	uart_pin_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
	uart_pin_conf.speed = GPIO_PIN_SPEED_HIGH;
	uart_pin_conf.pull = GPIO_PIN_NO_PULL_PUSH;
	pu_gpio_set_alt_function(GPIOA, 2, 0x7);
	pu_gpio_init(GPIOA, &uart_pin_conf);

	/* GPIO_PORT_A_PIN_3 will be used as RX  !!!*/
	uart_pin_conf.pin = 3;
	pu_gpio_set_alt_function(GPIOA, 3, 0x7);
	pu_gpio_init(GPIOA, &uart_pin_conf);
}
void uart_init(void)
{
	_PU_RCC_USART2_CLK_ENABLE();

	uart_handle2.Instance = USART_2;

	uart_handle2.Init.BaudRate = USART_BAUD_9600;
	uart_handle2.Init.WordLength = USART_WL_1S8B;
	uart_handle2.Init.StopBits = UART_STOP_BITS_1;
	uart_handle2.Init.Parity = UART_PARITY_NONE;
	uart_handle2.Init.Mode = UART_MODE_TX_RX;
	uart_handle2.Init.OverSampling = USART_OVER16_ENABLE;

	/* fill out the application callbacks */
	uart_handle2.tx_cmp_cb = app_tx_cmp_callback;
	uart_handle2.rx_cmp_cb = app_rx_cmp_callback;

	pu_uart_init(&uart_handle2);
}

/*=====================================================	
 *	
 * 					SPI STUFF
 * 
 * ==================================================== */

/* Initialize gpio pins for SPI functionality */
void spi_gpio_init(void)
{
	gpio_pin_conf_t spi_conf;

	_HAL_RCC_GPIOB_CLK_ENABLE();

	/* Configure GPIOB_PIN_13 for spi CLK functionality */
	spi_conf.pin = SPI_CLK_PIN;
	spi_conf.mode = GPIO_PIN_ALT_FUN_MODE;
	spi_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
	spi_conf.pull = GPIO_PIN_PULL_DOWN;
	spi_conf.speed = GPIO_PIN_SPEED_MEDIUM;

	pu_gpio_set_alt_function(GPIOB, SPI_CLK_PIN, GPIO_PIN_AF5_SPI2);
	pu_gpio_init(GPIOB, &spi_conf);

	/* configure GPIOB_PIN_14 for SPI MISO functionality */
	spi_conf.pin = SPI_MISO_PIN;
	spi_conf.pull = GPIO_PIN_PULL_UP;
	pu_gpio_set_alt_function(GPIOB, SPI_MISO_PIN, GPIO_PIN_AF5_SPI2);
	pu_gpio_init(GPIOB, &spi_conf);

	/* configure GPIOB_PIN_15 for SPI MOSI functionality */
	spi_conf.pin = SPI_MOSI_PIN;
	spi_conf.pull = GPIO_PIN_PULL_UP;
	pu_gpio_set_alt_function(GPIOB, SPI_MOSI_PIN, GPIO_PIN_AF5_SPI2);
	pu_gpio_init(GPIOB, &spi_conf);
}

void user_button_init(void)
{
	/* Configure USER Button interrupt*/
	_HAL_RCC_GPIOA_CLK_ENABLE();

	pu_gpio_configure_interrupt(GPIO_BUTTON_PIN, INT_FALLING_EDGE);
	pu_gpio_enable_interrupt(GPIO_BUTTON_PIN, EXTI0_IRQn);
}

void spi_init(void)
{
	/*
		- Using SPI2 device, running @500kHz, 
		- data format is 8bit MSB first
		- SPI mode = 1, because CPO=0 and CPHASE=1.
	*/

	_PU_RCC_SPI2_CLK_ENABLE();

	/* fill up the handle structure */
	spi_handle.Instance = SPI_2;
	spi_handle.Init.BaudRatePrescaler = PU_SPI_REG_CR1_BR_PCLK_DIV_32; //16MHz / 32 = 500kHz
	spi_handle.Init.Direction = PU_SPI_ENABLE_2_LINE_UNI_DIR;
	spi_handle.Init.CLKPhase = PU_SPI_SECOND_CLOCK_TRANS;
	spi_handle.Init.CLKPolarity = PU_SPI_CPOL_LOW;
	spi_handle.Init.DataSize = PU_SPI_8BIT_DF;
	spi_handle.Init.FirstBit = PU_SPI_TX_MSB_FIRST;
	spi_handle.Init.NSS = PU_SPI_SSM_ENABLE;
	spi_handle.Init.Mode = PU_SPI_MASTER_MODE_SEL;

	spi_handle.State = PU_SPI_STATE_READY;

	pu_spi_init(&spi_handle);

	NVIC_EnableIRQ(SPI2_IRQn);
}

/*=====================================================	
 *	
 * 					MAIN FUNCTIONS
 * 
 * ==================================================== */

int main(void)
{
	/* LED init */
	led_init();
	/* Blue button init */
	user_button_init();
	/* UART Init */
	uart_gpio_init();
	uart_init();
	/* SPI Init */
	spi_gpio_init();
	spi_init();
	/* UART Debug Init */
	//TODO: pu_debug_uart_init(DEBUG_USART_BAUD_9600);
	/* Enable the IRQs in the NVIC */
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_EnableIRQ(SPI2_IRQn);

	while (uart_handle2.tx_state != PU_UART_STATE_READY)
		;
	//Send the message
	uint8_t message1[] = "STM32F407 test";

	pu_uart_transmit(&uart_handle2, message1, sizeof(message1) - 1);

	while (1)
	{

		while (uart_handle2.rx_state != PU_UART_STATE_READY)
			;

		/*Recieve the message*/
		pu_uart_recieve(&uart_handle2, rx_buffer, 10);
	}
}

/*=====================================================	
 *	
 * 					IRQ Handlers
 * 
 * ==================================================== */

/**
  * @brief  This function handles SPI2 interrupt request.
  * @param  None
  * @retval None
  */
void SPI2_IRQHandler(void)
{
	/* call the driver api to process this interrupt */
	pu_spi_irq_handler(&spi_handle);
}

/**
  * @brief  This function handles EXTI0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
	/* In the ISR, first clear out the sticky interrupt pending bit for this interrupt */
	pu_gpio_clear_interrupt(GPIO_BUTTON_PIN);
	led_turn_on(GPIOD, LED_BLUE);
	//Send the message
	//uart_printf("SPI master Application running ... \n");
}