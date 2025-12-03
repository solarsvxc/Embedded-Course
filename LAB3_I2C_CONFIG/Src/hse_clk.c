#include "hse_clk.h" 

#define HSE_EN      (1U<<16)
#define HSE_RE_FLAG (1U<<17)
#define PLL_EN      (1U<<24)
#define PLL_RE_FLAG (1U<<27)

void hse_init(void)
{
    /* enable hse with 25Mhz oscillicator */
    RCC->CR |= HSE_EN;
    while (!(RCC->CR & HSE_RE_FLAG)){}
    /* enable pll */
    RCC->CR |= PLL_EN;
    while (!(RCC->CR & PLL_RE_FLAG)){}

    /* scale 1 -> 100Mhz */
    PWR->CR |= (1U<<14);
    PWR->CR |= (1U<<15);

    /* 01: HSE oscillator selected as system clock */
    RCC->CFGR |= (1U<<0);
    RCC->CFGR &=~(1U<<1);

    while ((RCC->CFGR & (3U<<2)) != (1U<<2)){}
}