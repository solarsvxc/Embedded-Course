#include <stm32f411xe.h>
#include <stdint.h>

void i2c_init(void);
void i2c_read_byte(char saddres,char maddress, char *data);
void i2c_read_burst_byte(char slave_address,char maddress,int n, char *data);
void i2c_write_burst(char slave_address, char maddress,int n, char *data);
