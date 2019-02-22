#include "led.h"

void led_init(void)
{
    gpio_pin_conf_t led_pin_conf;

    _HAL_RCC_GPIOD_CLK_ENABLE();

    led_pin_conf.pin = LED_ORANGE;
    led_pin_conf.mode = GPIO_PIN_OUTPUT_MODE;
    led_pin_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
    led_pin_conf.speed = GPIO_PIN_SPEED_MEDIUM;
    led_pin_conf.pull = GPIO_PIN_NO_PULL_PUSH;
    // led_pin_conf.pull = GPIO_PIN
    pu_gpio_init(GPIOD, &led_pin_conf);

    led_pin_conf.pin = LED_BLUE;
    pu_gpio_init(GPIOD, &led_pin_conf);

    led_pin_conf.pin = LED_RED;
    pu_gpio_init(GPIOD, &led_pin_conf);

    led_pin_conf.pin = LED_GREEN;
    pu_gpio_init(GPIOD, &led_pin_conf);
}

void led_turn_on(GPIO_TypeDef *GPIOx, uint16_t pin)
{
    pu_gpio_write_to_pin(GPIOx, pin, 1);
}

void led_turn_off(GPIO_TypeDef *GPIOx, uint16_t pin)
{
    pu_gpio_write_to_pin(GPIOx, pin, 0);
}

void led_toggle(GPIO_TypeDef *GPIOx, uint16_t pin)
{
    if (pu_gpio_read_from_pin(GPIOx, pin))
    {
        pu_gpio_write_to_pin(GPIOx, pin, 0);
    }
    else
    {
        pu_gpio_write_to_pin(GPIOx, pin, 1);
    }
}