/*    $$$$$$\   $$$$$$\  $$\         $$$$$$\  $$$$$$$\   $$$$$$\   $$\    $$\ $$\   $$\  $$$$$$\  
*    $$  __$$\ $$  __$$\ $$ |       $$  __$$\ $$  __$$\ $$  __$$\  $$ |   $$ |$$ |  $$ |$$  __$$\ 
*    $$ /  \__|$$ /  $$ |$$ |       $$ /  $$ |$$ |  $$ |$$ /  \__| $$ |   $$ |\$$\ $$  |$$ /  \__|
*    \$$$$$$\  $$ |  $$ |$$ |       $$$$$$$$ |$$$$$$$  |\$$$$$$\   \$$\  $$  | \$$$$  / $$ |      
*    \____$$\ $$ |  $$ |$$ |       $$  __$$ |$$  __$$<  \____$$\   \$$\$$  /  $$  $$<  $$ |      
*    $$\   $$ |$$ |  $$ |$$ |       $$ |  $$ |$$ |  $$ |$$\   $$ |   \$$$  /  $$  /\$$\ $$ |  $$\ 
*    \$$$$$$  | $$$$$$  |$$$$$$$$\  $$ |  $$ |$$ |  $$ |\$$$$$$  |    \$  /   $$ /  $$ |\$$$$$$  |
*    \______/  \______/ \________| \__|  \__|\__|  \__| \______/      \_/    \__|  \__| \______/ 
*
* ==== 26/11/2025 ==== just a walk through i2c ===== 
* 
*/

/* Specification 
* PB6 - SCL
* PB7 - SDA  
* Standard mode
* Master mode
*/

#include "i2c.h"

#define GPIOB_EN        (1U<<1)
#define FLAG_BUSY       (1U<<1)
#define FLAG_ADDR       (1U<<1)
#define FLAG_STARTBIT   (1U<<0)
#define FLAG_TxE        (1U<<7)
#define FLAG_RxNE       (1U<<6)
#define FLAG_BTF        (1U<<2)

void i2c_init(void)
{
    RCC->AHB1ENR |= GPIOB_EN;
    
    /* PB6 to alternative function mode */
    GPIOB->MODER &=~ (1U<<12);
    GPIOB->MODER |=  (1U<<13);
    /* PB7 to alternative function mode */
    GPIOB->MODER &=~ (1U<<14);
    GPIOB->MODER |=  (1U<<15);

    /* PB6 - PB7 output type to open-drain */
    GPIOB->OTYPER |= (1U<<6);
    GPIOB->OTYPER |= (1U<<7);

    /* PB6 - PB7 port type to pull-up */
    GPIOB->PUPDR &=~ (1U<<13);
    GPIOB->PUPDR |=  (1U<<12);
    GPIOB->PUPDR &=~ (1U<<15);
    GPIOB->PUPDR |=  (1U<<14);

    /* PB6 as line SCL in alternative function /\-0100-/\ */
    GPIOB->AFR[0] &=~(1U<<27); 
    GPIOB->AFR[0] |= (1U<<26); 
    GPIOB->AFR[0] &=~(1U<<25); 
    GPIOB->AFR[0] &=~(1U<<24); 

    /* PB7 as line SDA in alternative function /\-0100-/\ */
    GPIOB->AFR[0] &=~(1U<<31); 
    GPIOB->AFR[0] |= (1U<<30); 
    GPIOB->AFR[0] &=~(1U<<29); 
    GPIOB->AFR[0] &=~(1U<<28); 
    
    /* I2C enable */
    RCC->APB1ENR |=(1U<<21);

    /* SWRTS[15] enter software reset i2c bus, reset state-machine */
    I2C1->CR1 |= (1U<<15);
    /* SWRTS[15] get out reset */
    I2C1->CR1 &=~ (1U<<15);

    /* FREQ[5:0] set peripheral clock frequency EQUAL freq system, we's system running at 16MHz. /\== (16 = 00010000) ==/\ */
    I2C1->CR2 |= (1U<<4);

    /* CCR[11:0] Clock control register in Fm/Sm mode (Master mode), Standard mode */
    I2C1->CCR = 80;

    /* Standard mode max rise time */
    I2C1->TRISE = 17;

    /* Enable I2C module */
    I2C1->CR1 |=(1U<<0);
}

/**
 * @brief  read 1 byte 1 from slave 
 * 
 * @param slave_address [in] 
 * @param maddress      [in] memory address of slave
 * @param data          [in] 
 */
void i2c_read_byte(char slave_address,char maddress, char *data)
{
    volatile int tmp;

    while (I2C1->SR2 & FLAG_BUSY){}

    I2C1->CR1 |= (1U<<8);

    while (!(I2C1->SR1 & FLAG_STARTBIT)){}
    /* Write */
    I2C1->DR = slave_address << 1;

    while (!(I2C1->SR1 & FLAG_ADDR)){}

    tmp = I2C1->SR2;

    I2C1->DR = maddress;

    while (!(I2C1->SR1 & FLAG_TxE)){}

    /* enable start communication */
    I2C1->CR1 |= (1U<<8);

    while (!(I2C1->SR1 & FLAG_STARTBIT)){}
    
    /* Read */
    I2C1->DR = slave_address << 1 | 1;

    while (!(I2C1->SR1 & FLAG_ADDR)){}

    I2C1->CR1 &=~(1U<<10);

    tmp = I2C1->SR2;

    I2C1->CR1 |= (1U<<9);

    while (!(I2C1->SR1 & FLAG_RxNE)){}
    
    *data++ = I2C1->DR;
}

/**
 * @brief read array data from slave 
 * 
 * @param slave_address [in]
 * @param maddress      [in] memory address of slave
 * @param n             [in] size of array
 * @param data          [in]
 */
void i2c_read_burst_byte(char slave_address,char maddress, int n, char *data)
{
    volatile int tmp;
    
    /* wait busy is clear */
    while (I2C1->SR2 & FLAG_BUSY){}

    /* enable start communication */
    I2C1->CR1 |= (1U<<8);

    /* wait start flag is set*/
    while (!(I2C1->SR1 & FLAG_STARTBIT)){}

    /* Write */
    I2C1->DR = slave_address << 1;

    /* wait address sent */
    while (!(I2C1->SR1 & FLAG_ADDR)){}

    /* clear register sr2 */
    tmp = I2C1->SR2;

    /* write memory address to DataRegister */
    I2C1->DR = maddress;

    /* wait Transmit flag is set */
    while (!(I2C1->SR1 & FLAG_TxE)){}

    /* enable start communication  */
    I2C1->CR1 |= (1U<<8);

    /* wait start flag is set */
    while (!(I2C1->SR1 & FLAG_STARTBIT)){}
    
    /* Read */
    I2C1->DR = slave_address << 1 | 1;

    /* wait address sent */
    while (!(I2C1->SR1 & FLAG_ADDR)){}
    
    /* clear register sr2 */
    tmp = I2C1->SR2;

    /* enable bit ACK */
    I2C1->CR1 |= (1U<<10);

    while (n > 0U)
    {   
        /* if only data have one bit */
        if (n == 1U)
        {
            /* Disable bit acknowledge */
            I2C1->CR1 &=~(1U<<10);
            /* Generate stop */
            I2C1->CR1 |= (1U<<9);
            
            /* wait RXNE Flag is set */
            while (!(I2C1->SR1 & (FLAG_RxNE))){}
            /* get data from register */
            *data++ = I2C1->DR;
            break;
        } else
        {
            /* wait receiver flag is set */
            while (!(I2C1->SR1 & (FLAG_RxNE))){}
            /* read array data from DataRegister */
            *data++ = I2C1->DR;
            n--;
        }
    }
    
}
/**
 * @brief write array data to slave
 * 
 * @param slave_address [in]
 * @param maddress      [in] memory address slave
 * @param n             [in] size of data
 * @param data          [in] 
 */
void i2c_write_burst(char slave_address, char maddress,int n, char *data)
{
    volatile int tmp;
    
    /* wait busy flag is clear*/
    while (I2C1->SR2 & FLAG_BUSY){}

    /* enable start communication */
    I2C1->CR1 |= (1U<<8);
    
    /* wait start flag is set*/
    while (!(I2C1->SR1 & FLAG_STARTBIT)){}

    /* transmit slave address */
    I2C1->DR = slave_address << 1;

    /* wait address sent */
    while (!(I2C1->SR1 & FLAG_ADDR)){}

    /* clear address flag*/
    tmp = I2C1->SR2;

    /* wait flag trasmit is set  */
    while (!(I2C1->SR1 & (FLAG_TxE))){}

    /* send memory address */
    I2C1->DR = maddress;
    
    for (int i = 0; i < n; i++)
    {
        while (!(I2C1->SR1 & (FLAG_TxE))){}
        I2C1->DR = *data++;
    }

    /* wait transmit all data success */
    while (!(I2C1->SR1 & (FLAG_BTF))){}

    /* stop communication */
    I2C1->CR1 |= (1U<<9);
}