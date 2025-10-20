/*
 * spi.c
 *
 *  Created on: Oct 20, 2025
 *      Author: DUONG
 */
#include "spi.h"
#include <stdint.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define GPIOAEN (1U<<0)
#define GPIOBEN (1U<<1)
#define SPI1EN	(1U<<12)

#define SR_TXE  (1U<<1)     /* TXE buffer */
#define SR_RXNE (1U<<0)     /* RXNE buffer */
#define SR_BSY  (1U<<7)     /* BUSY flag */
/**
 * PA9 -> NSS/SS (we can chose PA4 to alternative function)
 * PA5 -> SCK
 * PA6 -> MISO
 * PA7 -> MOSI
 */

/*******************************************************************************
 * Code
 ******************************************************************************/

/**
 * @brief SPI1 init, we was set all pin need to be alternative function
 * 
 */
void spi1_init(void)
{
    /* Enable clock RCC to GPIOA */
    RCC->AHB1ENR |= GPIOAEN;

    /* PA 5 as alternative function */
    GPIOA->MODER   &=~(1U<<10); 
    GPIOA->MODER   |= (1U<<11); 
    
    /* PA 6 as alternative function */
    GPIOA->MODER   &=~(1U<<12); 
    GPIOA->MODER   |= (1U<<13);

    /* PA 7 as alternative function */
    GPIOA->MODER   &=~(1U<<14); 
    GPIOA->MODER   |= (1U<<15);
    
    /* PA 9 as output pin */
    GPIOA->MODER   |= (1U<<18); 
    GPIOA->MODER   &=~(1U<<19);

    /* PA5 to alternative function 0101 AFRL AF05 SPI1_SCK */
    GPIOA->AFR[0]    |= (1U<<20);
    GPIOA->AFR[0]    &=~(1U<<21);
    GPIOA->AFR[0]    |= (1U<<22);
    GPIOA->AFR[0]    &=~(1U<<23);

    /* PA6 to alternative function 0101 AFRL AF05 SPI1_MISO */
    GPIOA->AFR[0]    |= (1U<<24);
    GPIOA->AFR[0]    &=~(1U<<25);
    GPIOA->AFR[0]    |= (1U<<26);
    GPIOA->AFR[0]    &=~(1U<<27);

    /* PA7 to alternative function 0101 AFRL AF05 SPI1_MOSI */
    GPIOA->AFR[0]    |= (1U<<28);
    GPIOA->AFR[0]    &=~(1U<<29);
    GPIOA->AFR[0]    |= (1U<<30);
    GPIOA->AFR[0]    &=~(1U<<31);
}

/**
 * @brief config spi1 spec 8-bits data mode, MODE 1 1, Full duplex, MSB first
 * 
 */
void spi1_config(void)
{
    /* Enable access RCC to SPI1 */
    RCC->APB2ENR |= SPI1EN;

    /* Set: Baudrate equal fCLK/4 */
    SPI1->CR1   |=  (1U<<3);
    SPI1->CR1   &=~ (1U<<4);
    SPI1->CR1   &=~ (1U<<5);

    /* Set: CPOL to 1 and CPHAL to 1 */
    SPI1->CR1   |= (1U<<0);
    SPI1->CR1   |= (1U<<1);

    /* Set: RXONLY to Full Duplex*/
    SPI1->CR1   &=~(1U<<10);
    
    /* Set: SPI to MSBFIRST */
    SPI1->CR1   &=~(1U<<7);

    /* Set: MSTR to Master configuration */
    SPI1->CR1   |=(1U<<2);

    /* Set: DFF to 8-bits data mode */
    SPI1->CR1   &=~(1U<<11);

    /**
     * Set SSM to Software slave managerment enable
     * SSI Internal slave select
     */
    SPI1->CR1   |= (1U<<9);
    SPI1->CR1   |= (1U<<8);

    /* Enable SPI module */
    SPI1->CR1   |= (1U<<6);
}

/**
 * @brief data to transmit
 * 
 * @param [in] data 
 * @param [in] size 
 */
void spi1_transmit(uint8_t *data, uint32_t size)
{
    uint32_t j=0;
    uint8_t temp;

    while (j<size)
    {
        /* wait TX buffer is set */
        while (!(SPI1->SR & (SR_TXE))){}

        /* Write data to data register */
        SPI1->DR    = data[j];
        j++;
    }
    
    /* Wait TX buffer is set */
    while (!(SPI1->SR & (SR_TXE))){}

    /** 
    * Wait BSY flag is not busy
    * if it return 0, SPI is not busy
    * if it return 1, SPI is in communication or TX buffer isn't empty 
    */
    while (SPI1->SR & (SR_BSY)){}
    
    /* clear OVR flag */
    temp = SPI1->DR;
    temp = SPI1->SR;
}

/**
 * @brief 
 * 
 * @param data 
 * @param size 
 */
void spi1_receive(uint8_t *data, uint32_t size)
{
    uint8_t i=0;

    while (size)
    {
        /* Dummy data to Data register */
        SPI1->DR = 0;

        /* Wait for RXNE flag to be set */
        while (!(SPI1->SR & (SR_RXNE))){}
        
        /* Read data from data register */
        *data++ = SPI1->DR;
        size--;
    }
}

void cs_enable(void)
{
    GPIOA->ODR &=~ (1U<<9);
}

void cs_disable(void)
{
    GPIOA->ODR |= (1U<<9);
}