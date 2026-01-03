#include "stm32f411xe.h"
#include "spi.h"
#include "icm20948.h"
#include "nrf24.h"
#include <string.h>
#include "systick.h"

icm20948_raw_data_t raw_data;
Packet tx_packet;
uint8_t nrf_addr[5] = {0x31, 0x32, 0x33, 0x34, 0x35}; 


int main(void)
{
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));

    spi1_init();
    icm20948_init();

    nrf24_init();
    nrf24_tx_mode(nrf_addr, 80);

    while(1)
    {
        icm20948_read_accel_raw(&raw_data);

        tx_packet.x = raw_data.accel_x;
        tx_packet.y = raw_data.accel_y;
        tx_packet.z = raw_data.accel_z;
        strcpy(tx_packet.msg, "STM32");

        if (nrf24_transmit_packet(&tx_packet))
        {
            GPIOB->ODR ^= (1U<<12);
        }
        delay_ms(100);
    }
}