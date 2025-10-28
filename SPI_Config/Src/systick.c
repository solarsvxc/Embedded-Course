#include "systick.h"

#define CTRL_ENABLE		(1U<<0)
#define CTRL_CLCKSRC	(1U<<2)
#define CTRL_COUNTFLAG	(1U<<16)

/* load 16000 in 1ms*/
#define ONE_MSEC_LOAD	 16000
/* load 16 value in 1us*/
#define ONE_US_LOAD		 16


void delay_ms(uint32_t time_ms)
{
    /* Load the timer with number of clock cycles per millisecond*/
	SysTick->LOAD =  ONE_MSEC_LOAD - 1;

    /* Clear systick current value register*/
	SysTick->VAL = 0;

    /* Select internal clock source*/
	SysTick->CTRL = CTRL_CLCKSRC;

	/* Enable systick*/
	SysTick->CTRL |=CTRL_ENABLE;

	for(int i = 0; i < time_ms; i++)
	{
		while((SysTick->CTRL & CTRL_COUNTFLAG) == 0){}
	}

	/*Disable systick*/
	SysTick->CTRL = 0;
}

void delay_us(uint32_t time_us)
{
    /* Load the timer with number of clock cycles per millisecond*/
	SysTick->LOAD =  ONE_US_LOAD - 1;

    /* Clear systick current value register*/
	SysTick->VAL = 0;

    /* Select internal clock source*/
	SysTick->CTRL = CTRL_CLCKSRC;

	/* Enable systick*/
	SysTick->CTRL |=CTRL_ENABLE;

	for(int i = 0; i < time_us; i++)
	{
		while((SysTick->CTRL & CTRL_COUNTFLAG) == 0){}
	}

	/*Disable systick*/
	SysTick->CTRL = 0;
}
