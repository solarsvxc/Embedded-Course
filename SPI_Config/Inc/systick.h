
#ifndef SYSTICK_H_
#define SYSTICK_H_

#include <stdint.h>
#include "stm32f4xx.h"

void delay_ms(uint32_t time_ms);
void delay_us(uint32_t time_us);

#endif
