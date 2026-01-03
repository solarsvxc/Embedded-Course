#ifndef __SH1106_H__
#define __SH1106_H__

#include <stdint.h>
#include "i2c.h" 

#define SH1106_WIDTH    128
#define SH1106_HEIGHT   64
#define SH1106_I2C_ADDR 0x3C 

typedef enum {
    SH1106_COLOR_BLACK = 0x00, 
    SH1106_COLOR_WHITE = 0x01  
} SH1106_Color_t;

/* Các hàm giao tiếp chính */
void sh1106_init(void);
void sh1106_update(void);
void sh1106_fill(SH1106_Color_t color);
void sh1106_draw_pixel(uint8_t x, uint8_t y, SH1106_Color_t color);

#endif