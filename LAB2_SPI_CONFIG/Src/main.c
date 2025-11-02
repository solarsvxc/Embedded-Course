#include "gpio.h"
#include "systick.h"
#include "icm20948.h"
#include "tim.h"
#include "spi.h"
#include "stm32f4xx.h" 

#define GPIOB_EN   (1U<<1)

#define LED_GREEN              (1U<<3)
#define LED_BLUE               (1U<<4)
#define LED_WHITE              (1U<<5)
#define LED_ALERT              (1U<<6)

#define MAG_THRESHOLD          (10) 

icm20948_raw_data_t    g_raw_data;
icm20948_scaled_data_t g_scaled_data;

/**
 * 
 * @brief Khởi tạo các chân LED (PB3, PB4, PB5, PB6) làm Output.
 */
void ledb_init(void)
{
    RCC->AHB1ENR |= GPIOB_EN;

    GPIOB->MODER |= (1U<<6);
    GPIOB->MODER &=~(1U<<7);

    GPIOB->MODER |= (1U<<8);
    GPIOB->MODER &=~(1U<<9);

    GPIOB->MODER |= (1U<<10);
    GPIOB->MODER &=~(1U<<11);

    GPIOB->MODER |= (1U<<12);
    GPIOB->MODER &=~(1U<<13);

}

int main(void)
{
    /* Kích hoạt FPU (bắt buộc cho phép toán float) */
	SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
    __DSB();
    __ISB();

    ledb_init();
    spi1_init();
    spi1_config();
    
    if ( 0 != icm20948_init())
    {
        while(1)
        {
            GPIOB->ODR |= LED_ALERT;
        }
    }
    
    GPIOB->ODR &= ~(LED_GREEN | LED_BLUE | LED_WHITE);

    while (1)
    {
        icm20948_read_mag_raw(&g_raw_data);

        icm20948_convert_to_scaled(&g_raw_data, &g_scaled_data);

        if ((g_scaled_data.mag_x_ut > MAG_THRESHOLD) || (g_scaled_data.mag_x_ut < -MAG_THRESHOLD))
        {
            GPIOB->ODR |= LED_GREEN;
        }
        else
        {
            GPIOB->ODR &= ~LED_GREEN;
        }

        if ((g_scaled_data.mag_y_ut > MAG_THRESHOLD) || (g_scaled_data.mag_y_ut < -MAG_THRESHOLD))
        {
            GPIOB->ODR |= LED_BLUE;
        }
        else
        {
            GPIOB->ODR &= ~LED_BLUE;
        }

        if ((g_scaled_data.mag_z_ut > MAG_THRESHOLD) || (g_scaled_data.mag_z_ut < -MAG_THRESHOLD))
        {
            GPIOB->ODR |= LED_WHITE;
        }
        else
        {
            GPIOB->ODR &= ~LED_WHITE;
        }

        delay_ms(100);
        
    }
}
