#include "gpio_config.h"

#define GPIOAEN	(1U<<0)
#define GPIOCEN	(1U<<2)

#define LED_BS1	(1U<<1)  /*Bit Set Pin 1*/
#define LED_BR1	(1U<<17) /*Bit Reset Pin 1*/
#define BTN_PIN	(1U<<13)
#define LED_PIN	(1U<<1)

void led_init(void)
{
	/*Enable clock access to GPIOA*/
	RCC->AHB1ENR |= GPIOAEN;

	/*Set PA1 mode to output mode*/
	GPIOA->MODER |=(1U<<2);
	GPIOA->MODER &=~(1U<<3);
}


void led_on(void)
{
	/*Set PA1 high*/
	GPIOA->BSRR |=LED_BS1;
}

void led_off(void)
{
	/*Set PA1 low*/
	GPIOA->BSRR |=LED_BR1;

}

void led_toggle(void)
{
	/*Toggle PA1*/
	GPIOA->ODR ^=LED_PIN;
}

void button_init(void)
{
	/*Enable clock access to PORTC*/
	RCC->AHB1ENR |=GPIOCEN;

	/*Set PC13 as an input pin*/
	GPIOC->MODER &=~(1U<<26);
	GPIOC->MODER &=~(1U<<27);

}


bool get_btn_state(void)
{
	/*Note : BTN is active low*/
	/*Check if button is pressed*/
	if(GPIOC->IDR & BTN_PIN)
	{
		return false;
	}
	else
	{
		return true;
	}
}
