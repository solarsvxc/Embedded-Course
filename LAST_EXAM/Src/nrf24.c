#include "nrf24.h"
#include "spi.h"
#include "systick.h"

#define NRF_CSN_PIN     (1U<<12)
#define NRF_CE_PIN      (1U<<2)

static void nrf_csn_low(void) { GPIOB->ODR &= ~NRF_CSN_PIN; }
static void nrf_csn_high(void) { GPIOB->ODR |= NRF_CSN_PIN; }
static void nrf_ce_low(void) { GPIOB->ODR &= ~NRF_CE_PIN; }
static void nrf_ce_high(void) { GPIOB->ODR |= NRF_CE_PIN; }

static void nrf_write_reg(uint8_t reg, uint8_t value)
{
    nrf_csn_low();
    spi2_transmit_receive(CMD_W_REGISTER | reg);
    spi2_transmit_receive(value);
    nrf_csn_high();
}

static void nrf_write_buf(uint8_t reg, uint8_t *data, uint8_t size)
{
    nrf_csn_low();
    spi2_transmit_receive(CMD_W_REGISTER | reg);
    spi2_transmit(data, size);
    nrf_csn_high();
}

static uint8_t nrf_read_reg(uint8_t reg)
{
    uint8_t data;
    nrf_csn_low();
    spi2_transmit_receive(CMD_R_REGISTER | reg);
    data = spi2_transmit_receive(0xFF);
    nrf_csn_high();
    return data;
}

void nrf24_init(void)
{
    RCC->AHB1ENR |= (1U<<1);

    GPIOB->MODER |= (1U<<24); 
    GPIOB->MODER &= ~(1U<<25);
    GPIOB->MODER |= (1U<<4);  
    GPIOB->MODER &= ~(1U<<5);

    nrf_csn_high();
    nrf_ce_low();

    spi2_init();
}

void nrf24_tx_mode(uint8_t *address, uint8_t channel)
{
    nrf_ce_low();

    nrf_write_reg(NRF_RF_CH, channel);
    nrf_write_buf(NRF_TX_ADDR, address, 5);
    nrf_write_buf(NRF_RX_ADDR_P0, address, 5);

    nrf_write_reg(NRF_RF_SETUP, 0x06); 
    nrf_write_reg(NRF_CONFIG, 0x0E);   

    delay_ms(2);
}

uint8_t nrf24_transmit_packet(Packet *pkt)
{
    nrf_write_reg(NRF_STATUS, 0x70); 
    
    nrf_csn_low();
    spi2_transmit_receive(CMD_FLUSH_TX);
    nrf_csn_high();

    nrf_csn_low();
    spi2_transmit_receive(CMD_W_TX_PAYLOAD);
    spi2_transmit((uint8_t*)pkt, sizeof(Packet));
    nrf_csn_high();

    nrf_ce_high();
    delay_us(10);
    nrf_ce_low();

    uint32_t timeout = 100000;
    while (timeout--)
    {
        uint8_t status = nrf_read_reg(NRF_STATUS);
        if (status & (1U<<5)) 
        {
            nrf_write_reg(NRF_STATUS, (1U<<5)); 
            return 1;
        }
        if (status & (1U<<4)) 
        {
            nrf_write_reg(NRF_STATUS, (1U<<4)); 
            return 0;
        }
    }
    return 0;
}