/*
 * tim.c
 *
 *  Created on: Aug 26, 2025
 *      Author: DUONG
 */

#include "tim.h"
// ENABLE TIM2 default is set bit 0 || 1 is enable

#define TIM2EN (1U<<0)
#define CR1_EN (1U<<0)

void tim2_1hz_init(void){
        /* enable clock to tim2 */
        RCC->AHB1ENR |= TIM2EN;
        // 0 -> 1599 = 1600 value
        TIM2->PSC = 1600-1;
        // 0 -> 9999 = 10000 value counted = 10000 ticks in  = 1s
        TIM2->ARR = 10000-1;
        // clear counter value register
        TIM2->CNT = 0;
        // enable timer
        TIM2->CR1=CR1_EN;
}
