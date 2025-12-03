#include "i2c.h"
#include "sh1106.h"
#include "systick.h" 
/* 128 * 64 = 1024 bytes  */
static uint8_t sh1106_buffer[SH1106_WIDTH * SH1106_HEIGHT / 8];

static void write_commands(uint8_t cmd)
{
    i2c_write_burst(SH1106_I2C_ADDR,0x00,1,(char *)&cmd);
}

static void write_data(char *data,uint16_t size)
{
    i2c_write_burst(SH1106_I2C_ADDR,0x40,size,(char *)data);
}

void sh1106_update(void) 
{
    for (uint8_t page = 0; page < 8; page++) 
    {
        // 1. Chọn Page (0-7)
        write_commands(0xB0 + page);

        // 2. Chọn cột bắt đầu
        // SH1106 có 132 cột, màn 128 pixel nằm ở giữa.
        // Ta cần bỏ qua 2 cột đầu tiên (Cột 0 và 1) -> Bắt đầu ghi từ cột 2.
        // Cấu trúc lệnh set cột thấp (4 bit cuối): 0x00 + giá trị
        write_commands(0x00 + 0x02); // Set Lower Column Address (Offset = 2)
        
        // Cấu trúc lệnh set cột cao (4 bit đầu): 0x10 + giá trị
        write_commands(0x10);        // Set Higher Column Address (0)

        // 3. Ghi 128 byte dữ liệu của Page đó
        write_data(&sh1106_buffer[SH1106_WIDTH * page], SH1106_WIDTH);
    }
}

void sh1106_fill(SH1106_Color_t color) {
    uint8_t fill_byte = (color == SH1106_COLOR_WHITE) ? 0xFF : 0x00;
    for (int i = 0; i < sizeof(sh1106_buffer); i++) {
        sh1106_buffer[i] = fill_byte;
    }
}

void sh1106_init(void)
{
    delay_ms(1000);
    /* 0xAE: OFF */
    write_commands(0xAE);
    /* set ratio clock freq */
    write_commands(0xD5);
    /*  */
    write_commands(0x80);
    /* set multiplex ratio */
    write_commands(0xA8);
    /* set duty 1/64 */
    write_commands(0x3F);
    /* display offset screen */
    write_commands(0xD3);
    /* no collum offset */
    write_commands(0x00); 
    /* Set Display Start Line (0) */
    write_commands(0x40); 
    /* DC-DC Control Mode Set */
    write_commands(0xAD);
    /* DC-DC ON/OFF Mode Set ON */
    write_commands(0x8B);
    /* set segment right rotation */
    write_commands(0xA0);
    /* set scan direction */
    write_commands(0xC8);
    /* set common pads */
    write_commands(0xDA);
    /* sequential */
    write_commands(0x12);
    /* set constract screen*/
    write_commands(0x81);
    /* set brightness */
    write_commands(0xFF); 
    /* set pre-charge */
    write_commands(0xD9);
    /* set pre-chage period data */
    write_commands(0x1F);
    /* set vcom level */
    write_commands(0xD8);
    /* VCOM = β X VREF = ( β + A[7:0] X 0.006415) X VREF */
    write_commands(0x40);
    /* set entire display */
    write_commands(0xA4);
    /* set reverse color ? */
    write_commands(0xA6);

    /* ON display */
    write_commands(0xAF);
    sh1106_fill(SH1106_COLOR_WHITE);
    sh1106_update();
}

void sh1106_draw_pixel(uint8_t x, uint8_t y, SH1106_Color_t color)
{
    if (x > SH1106_WIDTH || y > SH1106_HEIGHT)
    {
        return;
    }

    
    
}
