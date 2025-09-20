/**
 * @file main.c
 * @author duong nguyen nguyen
 * @brief  stm32f411 blink 3 led with 2 button 
 * @version 0.1
 * @date 2025-09-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <stdint.h>

#define PERIPH_BASE 			(0x40000000UL)							        /*!< Peripheral base   		 address in the alias region */
#define AHB1PERIPH_BASE         (PERIPH_BASE + 0x00020000UL)			        /*!< Peripheral base, 		Address offset: 0x00020000UL */
#define GPIOA_BASE              (AHB1PERIPH_BASE + 0x0000UL)			        /*!< GPIOA base address,  	    Address offset: 0x00000UL */
#define RCC_BASE                (AHB1PERIPH_BASE + 0x3800UL)			        /*!< RCC base address, 		    Address offset:   0x3800UL*/
#define RCC_AHB1ENR				(*(uint32_t *)(RCC_BASE + 0x30UL))		        /*!< RCC AHB1ENR address	    Address offset:    0x30UL */
#define GPIOA_MODER             (*(uint32_t *)(GPIOA_BASE + 0x00UL))            /*!< GPIOx_MODER   register     Address offset:    0x00UL */
#define GPIOA_ODR				(*(uint32_t *)(GPIOA_BASE + 0x14UL))            /*!< GPIOx_ODR     register 	Address offset:    0x14UL */
#define GPIOA_IDR               (*(uint32_t *)(GPIOA_BASE + 0x10UL))            /*!< GPIOx_IDR     register 	Address offset:    0x10UL */
#define GPIOA_PUPDR             (*(uint32_t *)(GPIOA_BASE + 0x0CUL))            /*!< GPIOx_PUPDR   register	    Address offset:    0x0CUL */
#define GPIOA_ENR               (1U<<0)                                         /*!< shift 1 to bit 0	*/
#define PA1_LED_GREEN           (1U<<1)                                         /*!< shift 1 to bit 1   */
#define PA2_LED_RED             (1U<<2)                                         /*!< shift 1 to bit 2	*/
#define PA3_LED_WHITE           (1U<<3)                                         /*!< shift 1 to bit 3	*/
#define PA4_BTN_1               (1U<<4)                                         /*!< shift 1 to bit 4	*/
#define PA5_BTN_2               (1U<<5)                                         /*!< shift 1 to bit 5	*/
/* Found this information: https://developer.arm.com/documentation/dui0552/a/cortex-m3-peripherals/system-timer--systick */
#define SYST_CSR                (*(uint32_t *)(0xE000E010UL))                   /*!< SysTick Control and Status Register         */
#define SYST_RVR                (*(uint32_t *)(0xE000E014UL))                   /*!< SysTick Reload Value Register               */
#define SYST_CVR                (*(uint32_t *)(0xE000E018UL))                   /*!< SysTick Current Value Register              */
/*By default, the frequency of the MCU is 16Mhz in 1ms*/
#define CTRL_ENABLE		        (1U<<0)                                         /*!<    */
#define CTRL_CLCKSRC	        (1U<<2)                                         /*!<    */        
#define CTRL_COUNTFLAG	        (1U<<16)                                        /*!<    */    

/**
 * @brief init button as input mode
 * @param void 
 */
void button_init(void)
{
/*
*   GPIOx_MODER (rm0383)
*   00: Input (reset state)
*   01: General purpose output mode
*   10: Alternate function mode
*   11: Analog mode
* * We init pin as Input mode
*/
    GPIOA_MODER &= ~(1U<<8); 
    GPIOA_MODER &= ~(1U<<9); 
    //Set as input mode
    GPIOA_MODER &= ~(1U<<10); 
    GPIOA_MODER &= ~(1U<<11); 
/*
*   GPIOx_PUPDR (rm0383)
*   00: No pull-up, pull-down
*   01: Pull-up
*   10: Pull-down
*   11: Reserved
* * We init pin button as pull-down (by button)
*/
    //pin 4
    GPIOA_PUPDR &= ~(1U<<8);
    GPIOA_PUPDR |=  (1U<<9);
    //pin 5
    GPIOA_PUPDR &= ~(1U<<10);
    GPIOA_PUPDR |= (1U<<11);
}

/**
 * @brief init led pin 1, pin 2, pin 3 as output mode
 * @param void 
 */
void led_init(void)
{
/*
*   GPIOx_MODER (rm0383)
*   00: Input (reset state)
*   01: General purpose output mode
*   10: Alternate function mode
*   11: Analog mode
* * We init pin as General purpose output mode
*/
    //PIN 1
    GPIOA_MODER |= (1U<<2); 
    GPIOA_MODER &= ~(1U<<3); 
    //PIN 2
    GPIOA_MODER |= (1U<<4); 
    GPIOA_MODER &= ~(1U<<5); 
    //PIN 3
    GPIOA_MODER |= (1U<<6); 
    GPIOA_MODER &= ~(1U<<7); 
}

/**
 * @brief delay function using Systick Timer 
 * 
 * @param delayms 
 * @param one_msec_load 
 */
void delay_ms(uint32_t delayms, unsigned int  one_msec_load )
{
    SYST_RVR = one_msec_load - 1;
    SYST_CVR = 0;
    SYST_CSR = CTRL_CLCKSRC;
    SYST_CSR |= CTRL_ENABLE;
    // delay function
    for (unsigned int i = 0; i < delayms; i++)
    {
        while ((SYST_CSR & CTRL_COUNTFLAG) == 0);
    }
}

/**
 * @brief 
 * 
 * @param pinMask 
 */
void led_on(uint32_t pinMask)
{
    //toggle ^ (reverse state of pin)
    GPIOA_ODR ^= pinMask;
}

/**
 * @brief 
 * 
 * @param pinMask 
 */
void led_off(uint32_t pinMask)
{
    //clear bit field
    GPIOA_ODR &= ~pinMask;
}

/**
 * @brief 
 * 
 * @param pinMask 
 */
void led_toggle(uint32_t pinMask)
{
    GPIOA_ODR ^= pinMask;
}

/**
 * @brief get pin state right now
 * 
 * @param pin_btn 
 * @return int 
 */
int btn_state(uint32_t pin_btn)
{
    if(GPIOA_IDR & pin_btn) 
    {
        return 0;
    } else
    {
        return 1;
    } 
}
/**
 * @brief main function
 * 
 * @return void 
 */
int main(void)
{
RCC_AHB1ENR |= GPIOA_ENR;                   /*!< Enable RCC clock access on AHB1 bus for GPIOA */
led_init();                                
button_init();                                
/**
 * @brief 
 * 
 */
while(1)
{
    volatile int btn_left_state = btn_state(PA4_BTN_1);
    volatile int btn_right_state = btn_state(PA5_BTN_2);

    if(btn_left_state == 1 && btn_right_state == 1)
    {
        led_toggle(PA1_LED_GREEN);
        delay_ms(3000,16000);
    } 
    else if(btn_left_state == 0 && btn_right_state == 0)
    {
        led_toggle(PA1_LED_GREEN);
        led_toggle(PA2_LED_RED);
        led_toggle(PA3_LED_WHITE);
    } 
    else if(btn_left_state == 0 && btn_right_state == 1) 
    {
        led_toggle(PA1_LED_GREEN);
        delay_ms(1000,16000);
    }
    else if(btn_left_state == 1 && btn_right_state == 0)
    {
    	led_off(PA1_LED_GREEN);
        led_toggle(PA3_LED_WHITE);
    }
}

}
