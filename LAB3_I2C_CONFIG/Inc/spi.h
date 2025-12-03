#ifndef SPI_H_
#define SPI_H_

#include "stm32f411xe.h"
#include <stdint.h>

/*******************************************************************************
 * API
 ******************************************************************************/

void spi1_init(void);
void spi1_config(void);

void spi1_transmit(uint8_t *p_data, uint32_t size);
void spi1_receive(uint8_t *p_data, uint32_t size);

void cs_enable(void);
void cs_disable(void);


#endif /* SPI_H_ */
