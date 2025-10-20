/*
 * spi.h
 *
 *  Created on: Oct 20, 2025
 *      Author: DUONG
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>
#include "stm32f411xe.h"

/*******************************************************************************
 * API
 ******************************************************************************/
void spi1_init(void);
void spi1_config(void);
void spi1_transmit(uint8_t *data, uint32_t size);
void spi1_receive(uint8_t *data, uint32_t size);
void cs_enable(void);
void cs_disable(void);


#endif /* SPI_H_ */
