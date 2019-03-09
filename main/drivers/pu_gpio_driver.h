/**
 * @file - gpio_driver.h
 * @author - Pustinek
 * @date - 29.11.2018
 * @description - API for GPIO settings and initialization
 */

#include <stm32f4xx.h>
#include <stdint.h>
/*GPIO Pin mode selection values*/

/* Macros to Enable Clock for diffrent GPIO ports in RCC register */
#define _HAL_RCC_GPIOA_CLK_ENABLE() (RCC->AHB1ENR |= (1 << 0))
#define _HAL_RCC_GPIOB_CLK_ENABLE() (RCC->AHB1ENR |= (1 << 1))
#define _HAL_RCC_GPIOC_CLK_ENABLE() (RCC->AHB1ENR |= (1 << 2))
#define _HAL_RCC_GPIOD_CLK_ENABLE() (RCC->AHB1ENR |= (1 << 3))
#define _HAL_RCC_GPIOE_CLK_ENABLE() (RCC->AHB1ENR |= (1 << 4))
#define _HAL_RCC_GPIOF_CLK_ENABLE() (RCC->AHB1ENR |= (1 << 5))
#define _HAL_RCC_GPIOG_CLK_ENABLE() (RCC->AHB1ENR |= (1 << 6))
#define _HAL_RCC_GPIOH_CLK_ENABLE() (RCC->AHB1ENR |= (1 << 7))

#define GPIO_PORT_A GPIOA
#define GPIO_PORT_B GPIOB
#define GPIO_PORT_C GPIOC
#define GPIO_PORT_D GPIOD
#define GPIO_PORT_E GPIOE

typedef enum GPIO_PIN_MODE
{
    GPIO_PIN_INPUT_MODE = 0x00,   /* GPIO Pin as General Purpose Input*/
    GPIO_PIN_OUTPUT_MODE = 0x01,  /* GPIO Pin as General Purpose Output*/
    GPIO_PIN_ALT_FUN_MODE = 0x02, /* GPIO Pin as Alternate function*/
    GPIO_PIN_AN_MODE = 0x03,      /* GPIO Pin as Analog*/

} GPIO_PIN_MODE;
/*GPIO OP Type*/
typedef enum GPIO_PIN_OP_TYPE
{
    GPIO_PIN_OP_TYPE_PUSHPULL = 0x00,
    GPIO_PIN_OP_TYPE_OPEN_DRAIN = 0x01,
} GPIO_PIN_OP_TYPE;

/*GPIO Speed*/
typedef enum GPIO_PIN_SPEED
{
    GPIO_PIN_SPEED_LOW = 0x00,
    GPIO_PIN_SPEED_MEDIUM = 0x01,
    GPIO_PIN_SPEED_HIGH = 0x02,
    GPIO_PIN_SPEED_VERYHIGH = 0x03,
} GPIO_PIN_SPEED;

typedef enum GPIO_PIN_PULL
{
    GPIO_PIN_NO_PULL_PUSH = 0x00,
    GPIO_PIN_PULL_UP = 0x01,
    GPIO_PIN_PULL_DOWN = 0x11,
} GPIO_PIN_PULL;

/* Interrupt Edge selection enum */
typedef enum
{
    INT_RISING_EDGE,
    INT_FALLING_EDGE,
    INT_RISING_FALLING_EDGE
} int_edge_sel_t;

/**
 * @brief GPIO pin configuration structure
 *         This structure will be filled and passed to driver by aplication to initialize the gpio pin
 * 
 */
typedef struct gpio_pin_conf_t
{
    uint16_t pin;
    uint32_t mode;
    uint32_t op_type;
    uint32_t pull;
    uint32_t speed;
    uint32_t alternate;
} gpio_pin_conf_t;

/*===============DRIVER EXPOSED APIs================*/
void pu_gpio_init(GPIO_TypeDef *GPIOx, gpio_pin_conf_t *gpio_pin_conf);

void pu_gpio_configure_pin_mode(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t mode);

void pu_gpio_configure_pin_otype(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t op_type);

void pu_gpio_configure_pin_speed(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t speed);

void pu_gpio_configure_pin_pupd(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t pupd);

void pu_gpio_set_alt_function(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint16_t alt_fun_value);

uint8_t pu_gpio_read_from_pin(GPIO_TypeDef *GPIOx, uint16_t pin_no);

void pu_gpio_write_to_pin(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint8_t val);

void pu_gpio_configure_interrupt(uint16_t pin_no, int_edge_sel_t edge_sel);

void pu_gpio_enable_interrupt(uint16_t pin_no, IRQn_Type irq_no);

void pu_gpio_clear_interrupt(uint16_t pin);

