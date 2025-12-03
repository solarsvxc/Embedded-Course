#include "i2c.h"
#include "sh1106.h"

int main(void) 
{
    i2c_init();
    
    sh1106_init();
    sh1106_update();

    while(1) {

    }
}